#include "modules/treeguard/TreeGuardModule.h"

#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include <Arduino.h>
#include <Wire.h>

#include "mesh/MeshService.h"
#include "meshtastic/mesh.pb.h" // PortNum + MeshPacket types
#include "sleep.h" // Meshtastic deep sleep integration
#include <cstring>

extern MeshService *service; // provided by Meshtastic core

#include "modules/treeguard/TreeGuardPins.h"
#include "modules/treeguard/classifier.h"
#include "modules/treeguard/fft/fft_config.h"

// Vendored IMU
#include "modules/treeguard/MPU6050/I2Cdev.h"
#include "modules/treeguard/MPU6050/MPU6050.h"

#ifdef USE_MAX1704X
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#endif

#ifndef TG_ACCEL_SAMPLE_RATE_HZ
#define TG_ACCEL_SAMPLE_RATE_HZ 333
#endif

static MPU6050 s_mpu;
#ifdef USE_MAX1704X
static SFE_MAX1704X s_gauge;
#endif

static float g_ax[FFT_N];
static float g_ay[FFT_N];
static float g_az[FFT_N];

static inline void TG_warnIfIntNotRTC()
{
    if (!rtc_gpio_is_valid_gpio((gpio_num_t)TG_INT_PIN)) {
        Serial.println("[TreeGuard] WARNING: TG_INT_PIN not RTC-capable; EXT0 wake disabled, timer-only wake will be used.");
    }
}

TreeGuardModule::TreeGuardModule()
    : SinglePortModule("treeguard", meshtastic_PortNum_TEXT_MESSAGE_APP), // <- OK to use TEXT_MESSAGE_APP
      OSThread("TreeGuard")
{
    Serial.println("[TreeGuard] module constructed");
    // Register observer for deep sleep preparation
    notifyDeepSleepObserver.observe(&notifyDeepSleep);
    // Delay first execution to allow system to fully initialize (30 seconds)
    setIntervalFromNow(30 * 1000);
}

void TreeGuardModule::sendText(const char *message)
{
    if (!service || !message)
        return;

    meshtastic_MeshPacket pkt = meshtastic_MeshPacket_init_default;
    pkt.to = 0; // broadcast
    pkt.want_ack = false;

    // Send as a TEXT message directly in decoded fields (no Data/DataApp wrapper)
    pkt.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;

    const size_t maxlen = sizeof(pkt.decoded.payload.bytes);
    const size_t len = strnlen(message, maxlen);
    memcpy(pkt.decoded.payload.bytes, message, len);
    pkt.decoded.payload.size = (pb_size_t)len;

    service->sendToMesh(&pkt);
}

void TreeGuardModule::sampleAccelBlock()
{
    const unsigned long dt_us = 1000000UL / TG_ACCEL_SAMPLE_RATE_HZ;
    unsigned long t_next = micros();

    for (int i = 0; i < FFT_N; ++i) {
        int16_t ax, ay, az, gx, gy, gz;
        s_mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        g_ax[i] = ax;
        g_ay[i] = ay;
        g_az[i] = az;

        t_next += dt_us;
        long wait = (long)(t_next - micros());
        if (wait > 0)
            delayMicroseconds(wait);
    }
}

void TreeGuardModule::processTimerWake()
{
    float v = 0.f, soc = 0.f, tC = -100.f;
#ifdef USE_MAX1704X
    if (s_gauge.begin(Wire)) {
        v = s_gauge.getVoltage();
        soc = s_gauge.getSOC();
    }
#endif
    s_mpu.initialize();
    if (s_mpu.testConnection()) {
        tC = s_mpu.getTemperature() / 340.00f + 36.53f;
    }

    char msg[128];
    snprintf(msg, sizeof(msg), "STAT,V:%.2f,SOC:%.1f,T:%.1f", v, soc, tC);
    sendText(msg);
}

void TreeGuardModule::processVibrationWake()
{
    s_mpu.initialize();
    if (!s_mpu.testConnection()) {
        sendText("ALARM,MPU6050_FAIL");
        return;
    }
    s_mpu.setSleepEnabled(false);
    delay(100);
    s_mpu.setStandbyXGyroEnabled(true);
    s_mpu.setStandbyYGyroEnabled(true);
    s_mpu.setStandbyZGyroEnabled(true);

    sampleAccelBlock();

    Features f{};
    String result = classifyBufferedData(g_ax, g_ay, g_az, (int)FFT_N, &f);

    float v = 0.f, soc = 0.f;
#ifdef USE_MAX1704X
    if (s_gauge.begin(Wire)) {
        v = s_gauge.getVoltage();
        soc = s_gauge.getSOC();
    }
#endif

    char msg[224];
    snprintf(msg, sizeof(msg), "TG,%s,hfe:%.2f,r1:%.2f,r2:%.2f,sc:%d,V:%.2f,SOC:%.1f", result.c_str(), f.hf_energy,
             f.fft_0_25_ratio, f.fft_125_200_ratio, f.strike_count, v, soc);
    sendText(msg);
}

int TreeGuardModule::prepareDeepSleep(void *unused)
{
    // This observer callback is called by Meshtastic before entering deep sleep
    // We use it to configure the EXT0 wake source for GPIO interrupt
    Serial.println("[TreeGuard] Preparing for deep sleep...");
    
    // Configure GPIO 7 for EXT0 wake (vibration sensor interrupt)
    if (rtc_gpio_is_valid_gpio((gpio_num_t)TG_INT_PIN)) {
        pinMode(TG_INT_PIN, INPUT_PULLDOWN);
        rtc_gpio_pullup_dis((gpio_num_t)TG_INT_PIN);
        rtc_gpio_pulldown_en((gpio_num_t)TG_INT_PIN);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)TG_INT_PIN, 1); // Wake on HIGH
        Serial.println("[TreeGuard] EXT0 wake enabled on GPIO 7");
    } else {
        Serial.println("[TreeGuard] WARNING: GPIO 7 not RTC-capable, timer-only wake");
    }
    
    // Power down VEXT sensors
    TG_vextOff();
    delay(TG_VEXT_POWERDOWN_DELAY_MS);
    Serial.println("[TreeGuard] VEXT powered down");
    
    // Note: Wire.end() is handled by Meshtastic's doDeepSleep() at line 338 of sleep.cpp
    // We don't call it here to avoid conflicts
    
    return 0; // Must return 0
}

void TreeGuardModule::goToDeepSleep()
{
    Serial.println("[TreeGuard] Entering deep sleep for 72 hours or until vibration detected");
    Serial.flush();
    delay(100);
    
    // Use Meshtastic's doDeepSleep() which will:
    // 1. Call our prepareDeepSleep() observer to configure EXT0 and power down VEXT
    // 2. Handle all the standard cleanup (I2C, Bluetooth, screen, etc.)
    // 3. Configure timer wake for 72 hours
    // 4. Enter deep sleep via cpuDeepSleep()
    doDeepSleep(TG_TIMER_WAKE_SECONDS * 1000ULL, false, false);
    
    // Never returns - device will reset on wake
}

int32_t TreeGuardModule::runOnce()
{
    sendText("TG_BOOT"); // quick end-to-end radio check
    static bool ran = false;
    if (ran)
        return 0;
    ran = true;
    Serial.println("[TreeGuard] runOnce() start");

    TG_warnIfIntNotRTC();

    TG_vextOn();
    delay(TG_VEXT_POWERUP_DELAY_MS); // 200ms for sensor stabilization
    Wire.begin(TG_I2C_SDA, TG_I2C_SCL);

    const auto wake = esp_sleep_get_wakeup_cause();
    if (wake == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("[TreeGuard] Wake: EXT0 (shake)");
        processVibrationWake();
    } else if (wake == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println("[TreeGuard] Wake: TIMER");
        processTimerWake();
    } else {
        Serial.println("[TreeGuard] Wake: power-on/other -> STATUS");
        processTimerWake();
    }

    delay(4000); // let radio TX
    goToDeepSleep();
    // We never get here (deep sleep), but return type required.
    return 0;
}
