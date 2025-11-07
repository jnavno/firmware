#include "modules/treeguard/TreeGuardModule.h"

#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include <Arduino.h>
#include <Wire.h>

#include "mesh/MeshService.h"
#include "meshtastic/mesh.pb.h" // PortNum + MeshPacket types
#include "sleep.h"              // Meshtastic deep sleep integration
#include <cstring>

#include "NodeDB.h"
#include "Router.h" // for extern router
#include "configuration.h"
#include "meshUtils.h"       // for generatePacketId()
extern MeshService *service; // provided by Meshtastic core
extern Router *router;

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

// classifier window (what the FFT-based code expects)
static constexpr uint16_t TG_CLS_WINDOW = FFT_N; // from classifier (1024)

static float g_ax[FFT_N];
static float g_ay[FFT_N];
static float g_az[FFT_N];

static constexpr uint8_t TG_CH_ALERTAS = 3;
static constexpr uint8_t TG_CH_REDUVERD = 4;
static constexpr uint8_t TG_CH_ADMINRED = 2; // status / IoT / debug

static inline void TG_warnIfIntNotRTC()
{
    if (!rtc_gpio_is_valid_gpio((gpio_num_t)TG_INT_PIN)) {
        Serial.println("[TreeGuard] WARNING: TG_INT_PIN not RTC-capable; EXT0 wake disabled, timer-only wake will be used.");
    }
}

TreeGuardModule::TreeGuardModule()
    : ProtobufModule<_meshtastic_TreeGuardMetrics>("treeguard", meshtastic_PortNum_TEXT_MESSAGE_APP, &meshtastic_TreeGuardMetrics_msg), // <- OK to use TEXT_MESSAGE_APP
      OSThread("TreeGuard")
{
    Serial.println("[TreeGuard] module constructed");
    // Register observer for deep sleep preparation
    notifyDeepSleepObserver.observe(&notifyDeepSleep);

    // Always wait 30 seconds for system initialization
    // Classification and message sending happens within this window
    Serial.println("[TreeGuard] Delaying 5 seconds for system initialization");
    setIntervalFromNow(5 * 1000);
}

void TreeGuardModule::sendText(const char *message, uint8_t channel=TG_CH_ALERTAS )
{
    if (!message || !service) {
        Serial.println("[TreeGuard] ERROR: message/service is NULL");
        return;
    }

    Serial.print("[TreeGuard] Sending message: ");
    Serial.println(message);

    // Allocate a packet structure
    meshtastic_MeshPacket pkt = meshtastic_MeshPacket_init_default;

    // --- BASIC PACKET ROUTING ---
    pkt.to = 0xFFFFFFFF; // broadcast to all
    pkt.from = 0;        // filled automatically on TX
    pkt.hop_limit = 3;   // standard for general messages
    pkt.want_ack = false;
    pkt.priority = meshtastic_MeshPacket_Priority_DEFAULT;
    pkt.id = generatePacketId(); // unique ID from core utility

    // --- CHANNEL ---
    // Pull current node channel index from NodeDB
    auto *node = nodeDB->getMeshNode(nodeDB->getNodeNum());
    pkt.channel = channel; // use 0 if unknown

    // --- PAYLOAD ---
    pkt.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    pkt.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;

    const size_t maxlen = sizeof(pkt.decoded.payload.bytes);
    size_t len = strnlen(message, maxlen);
    memcpy(pkt.decoded.payload.bytes, message, len);
    pkt.decoded.payload.size = (pb_size_t)len;

    // --- SEND INTO MESH ---
    service->sendToMesh(packetPool.allocCopy(pkt), RX_SRC_LOCAL, false);

    Serial.println("[TreeGuard] Message queued for transmission");
}

void TreeGuardModule::sendProto(const _meshtastic_TreeGuardMetrics &msg, uint8_t channel=TG_CH_REDUVERD)
{
    if (!service) {
        Serial.println("[TreeGuard] ERROR: service is NULL");
        return;
    }

    Serial.printf("[TreeGuard] Sending Proto message: %d\n",msg.timestamp_ms);

    // Allocate a packet structure
    meshtastic_MeshPacket *pkt = allocDataProtobuf(msg);

    // --- BASIC PACKET ROUTING ---
    // pkt->to = 0xFFFFFFFF; // broadcast to all
    // pkt->from = 0;        // filled automatically on TX
    // pkt->hop_limit = 3;   // standard for general messages
    // pkt->want_ack = false;
    // pkt->priority = meshtastic_MeshPacket_Priority_DEFAULT;
    // pkt->id = generatePacketId(); // unique ID from core utility

    // --- CHANNEL ---
    // Pull current node channel index from NodeDB
    // auto *node = nodeDB->getMeshNode(nodeDB->getNodeNum());
    pkt->channel = channel; // use 0 if unknown


    // --- SEND INTO MESH ---
    service->sendToMesh(pkt, RX_SRC_LOCAL, false);

    Serial.println("[TreeGuard] Proto Message queued for transmission");
}

bool TreeGuardModule::captureVibration333(TG_VibCapture &cap)
{
    Serial.println("[TreeGuard] VIB: sampling (12s @ 333Hz) ...");

    const uint32_t interval_us = 1000000UL / TG_SAMPLE_HZ; // ≈ 3003 us
    uint32_t next_ts = micros();

    for (uint16_t i = 0; i < TG_NUM_SAMPLES; ++i) {
        int16_t ax, ay, az, gx, gy, gz;

        // read raw accel+gyro (MPU6050 style API)
        s_mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        cap.ax[i] = ax;
        cap.ay[i] = ay;
        cap.az[i] = az;

        // schedule next sample
        next_ts += interval_us;
        // busy-wait to keep timing tight
        while ((int32_t)(micros() - next_ts) < 0) {
            // optional: yield() if you trust it not to jitter too much
        }
    }

    Serial.println("[TreeGuard] VIB: sampling done.");
    return true;
}

String TreeGuardModule::classifyFromCapture(const TG_VibCapture &cap, Features *outFeatures, float &hf, float &r1, float &r2,
                                            int &sc)
{
    // static to avoid big stack frames
    static float fx[FFT_N];
    static float fy[FFT_N];
    static float fz[FFT_N];

    // pick the last 1024 samples from the 12s capture
    uint16_t start = 0;
    if (TG_NUM_SAMPLES > FFT_N) {
        start = TG_NUM_SAMPLES - FFT_N; // 3996 - 1024 = 2972
    }

    for (uint16_t i = 0; i < FFT_N; ++i) {
        fx[i] = (float)cap.ax[start + i];
        fy[i] = (float)cap.ay[start + i];
        fz[i] = (float)cap.az[start + i];
    }

    Features localFeat;
    Features *featPtr = outFeatures ? outFeatures : &localFeat;

    String label = classifyBufferedData(fx, fy, fz, FFT_N, featPtr);

    // expose the 3 values to format the message
    hf = featPtr->hf_energy;
    r1 = featPtr->fft_0_25_ratio;
    r2 = featPtr->fft_125_200_ratio;
    sc = featPtr->strike_count;

    return label;
}

void TreeGuardModule::sampleAccelBlock()
{
    const unsigned long dt_us = 1000000UL / TG_ACCEL_SAMPLE_RATE_HZ;
    unsigned long t_next = micros();
    // only sample as many as we actually have space for
    const int N = min((int)TG_ACCEL_NUM_SAMPLES, (int)FFT_N);
    for (int i = 0; i < N; i++) {
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
    sendText(msg, TG_CH_ALERTAS);
}

void TreeGuardModule::processVibrationWake()
{
    Serial.println("[TreeGuard] VIB: start");
    s_mpu.initialize();
    if (!s_mpu.testConnection()) {
        sendText("ALARM,MPU6050_FAIL");
        return;
    }
    Serial.println("[TreeGuard] VIB: MPU ok");

    s_mpu.setSleepEnabled(false);
    delay(100);
    s_mpu.setStandbyXGyroEnabled(true);
    s_mpu.setStandbyYGyroEnabled(true);
    s_mpu.setStandbyZGyroEnabled(true);

    Serial.println("[TreeGuard] VIB: sampling...");
    sampleAccelBlock();
    Serial.println("[TreeGuard] VIB: sampling done, classify...");

    int N = min((int)TG_ACCEL_NUM_SAMPLES, (int)FFT_N);
    Features f{};
    String result = classifyBufferedData(g_ax, g_ay, g_az, N, &f);

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
    Serial.println("[TreeGuard] VIB: sent TG message");
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
    doDeepSleep(TG_TIMER_WAKE_SECONDS * 1000ULL, true, false);

    // Never returns - device will reset on wake
}

int32_t TreeGuardModule::runOnce()
{
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

        // 1. capture raw
        static TG_VibCapture cap;
        if (captureVibration333(cap)) {
            // 2. classify
            float hf, r1, r2;
            int sc;
            Features feats; // from classifier.h
            String label = classifyFromCapture(cap, &feats, hf, r1, r2, sc);

            // 3. format message
            char msg[160];
            snprintf(msg, sizeof(msg), "TG,%s,hfe:%.2f,r1:%.2f,r2:%.2f,sc:%d,V:0.00,SOC:0.0", label.c_str(), hf, r1, r2, sc);

            sendText(msg , TG_CH_ALERTAS);

//THIS IS JUST A TEST
            meshtastic_TreeGuardMetrics metrics = meshtastic_TreeGuardMetrics_init_zero;

            metrics.timestamp_ms = millis();
            metrics.strike_count = 42;
            metrics.offset_x_y = 0.15f;
            metrics.offset_y_z = 0.23f;
            metrics.max_y = 1.2f;
            metrics.max_z = 0.8f;
            metrics.fft_low_ratio = 0.35f;
            metrics.fft_high_ratio = 0.12f;
            metrics.hf_energy_ratio = 0.08f;
            metrics.battery_soc_percent = 87.5f;

            sendProto(metrics);

            Serial.println("[TreeGuard] VIB: sent TG message");
        } else {
            sendText("TG,⚠️ VIB_CAPTURE_FAIL,V:0.00,SOC:0.0", TG_CH_ADMINRED);
        }

        sendText("TG_BOOT");
    } else {
        Serial.println("[TreeGuard] Wake: power-on/other -> STATUS");
        sendText("STAT,V:0.00,SOC:0.0,T:27.0");
        sendText("TG_BOOT");
    }

    // 4. give radio a moment to tx (your queue was 16 entries)
    uint32_t t0 = millis();
    while ((millis() - t0) < 2000) {
        meshtastic_QueueStatus qs = router->getQueueStatus();
        if (qs.free == qs.maxlen) {
            Serial.printf("[TreeGuard] TX queue drained: %u / %u\n", qs.free, qs.maxlen);
            break;
        }
        // don't spam every 100 ms, print only first time
        static bool printed = false;
        if (!printed) {
            Serial.printf("[TreeGuard] TX queue: free=%u / %u (waiting)\n", qs.free, qs.maxlen);
            printed = true;
        }
        delay(100);
    }

    goToDeepSleep();
    // We never get here (deep sleep), but return type required.
    return 0;
}

bool TreeGuardModule::handleReceivedProtobuf(const meshtastic_MeshPacket &p, _meshtastic_TreeGuardMetrics *msg)
{
    return true;
}
