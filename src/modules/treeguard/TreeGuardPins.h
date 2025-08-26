#pragma once
#include <Arduino.h>

/*
  TreeGuard module-local pin map (Heltec Wireless Stick Lite V3 / ESP32-S3)
  -------------------------------------------------------------------------
  - Defaults are chosen for Heltec V3 hardware you’ve been using.
  - All values can be overridden from platformio.ini with -D macros, e.g.:
      build_flags =
        -DTG_INT_PIN=GPIO_NUM_7
        -DTG_I2C_SDA=GPIO_NUM_41
        -DTG_I2C_SCL=GPIO_NUM_42
        -DTG_VEXT_PIN=GPIO_NUM_36
  - IMPORTANT: EXT0 wake must use an **RTC-capable** GPIO on ESP32-S3.
    If TG_INT_PIN is not an RTC pin, EXT0/EXT1 wake won’t work (timer-only).
*/

/* ─────────────────────────────────────────────
   Power rail (VEXT is active-LOW on Heltec V3)
   ───────────────────────────────────────────── */
#ifndef TG_VEXT_PIN
#define TG_VEXT_PIN GPIO_NUM_36
#endif

/* ─────────────────────────────────────────────
   I2C for sensors
   ───────────────────────────────────────────── */
#ifndef TG_I2C_SDA
#define TG_I2C_SDA GPIO_NUM_41
#endif
#ifndef TG_I2C_SCL
#define TG_I2C_SCL GPIO_NUM_42
#endif

/* ─────────────────────────────────────────────
   Interrupt/wakeup from sensor (EXT0 target)
   NOTE: Must be an RTC-capable pin on ESP32-S3
   ───────────────────────────────────────────── */
#ifndef TG_INT_PIN
#define TG_INT_PIN GPIO_NUM_7
#endif

/* ─────────────────────────────────────────────
   Optional LEDs (only used if you reference them)
   ───────────────────────────────────────────── */
#ifndef TG_STATUS_LED_PIN
#define TG_STATUS_LED_PIN GPIO_NUM_4 // change to 34 if your PCB uses it
#endif
#ifndef TG_ALERT_LED_PIN
#define TG_ALERT_LED_PIN GPIO_NUM_5
#endif

/* ─────────────────────────────────────────────
   Optional SPI (only if you actually use it)
   ───────────────────────────────────────────── */
#ifndef TG_SD_CS_PIN
#define TG_SD_CS_PIN GPIO_NUM_19
#define TG_SD_MOSI_PIN GPIO_NUM_38
#define TG_SD_MISO_PIN GPIO_NUM_39
#define TG_SD_SCK_PIN GPIO_NUM_37
#endif

/* ─────────────────────────────────────────────
   Battery thresholds (module-local)
   ───────────────────────────────────────────── */
#ifndef TG_MIN_BATT_VOLTAGE
#define TG_MIN_BATT_VOLTAGE 3.3f
#endif
#ifndef TG_MAX_BATT_VOLTAGE
#define TG_MAX_BATT_VOLTAGE 4.2f
#endif
#ifndef TG_SOC_THRESHOLD
#define TG_SOC_THRESHOLD 20u
#endif

/* ─────────────────────────────────────────────
   Sampling config (module-local)
   Your classifier code locks sample count to FFT_N,
   so leave duration/rate here as soft defaults only.
   ───────────────────────────────────────────── */
#ifndef TG_ACCEL_SAMPLE_RATE_HZ
#define TG_ACCEL_SAMPLE_RATE_HZ 333
#endif
#ifndef TG_ACCEL_DURATION_SECONDS
#define TG_ACCEL_DURATION_SECONDS 12
#endif
#ifndef TG_ACCEL_NUM_SAMPLES
#define TG_ACCEL_NUM_SAMPLES (TG_ACCEL_SAMPLE_RATE_HZ * TG_ACCEL_DURATION_SECONDS)
#endif

#ifndef TG_NUM_BLOCKS
#define TG_NUM_BLOCKS 25 // 25 * 12s ≈ 5 minutes per session
#endif

/* If you really need runtime-tunable values, declare them here and
   define them ONCE in a .cpp (e.g., TreeGuardGlobals.cpp) */
extern int TG_SAMPLE_RATE; // Hz (optional; define if used)
extern int TG_SAMPLES;     // count (optional; define if used)

/* ─────────────────────────────────────────────
   Filenames (module-local)
   ───────────────────────────────────────────── */
#ifndef TG_SPIFFS_FILE
#define TG_SPIFFS_FILE "/data.csv"
#endif
#ifndef TG_CLASSIFICATION_FILE
#define TG_CLASSIFICATION_FILE "/classification_log.txt"
#endif
#ifndef TG_STATUS_FILE
#define TG_STATUS_FILE "/status_log.txt"
#endif
#ifndef TG_STATE_FILE
#define TG_STATE_FILE "/log_state.txt"
#endif

/* ─────────────────────────────────────────────
   Detection thresholds (module-local)
   ───────────────────────────────────────────── */
#ifndef TG_MPU6050_ADDR
#define TG_MPU6050_ADDR 0x68
#endif

#ifndef TG_SAW_MIN_HZ
#define TG_SAW_MIN_HZ 30.0f
#endif
#ifndef TG_SAW_MAX_HZ
#define TG_SAW_MAX_HZ 60.0f
#endif
#ifndef TG_CHAINSAW_MIN_HZ
#define TG_CHAINSAW_MIN_HZ 200.0f
#endif
#ifndef TG_CHAINSAW_MAX_HZ
#define TG_CHAINSAW_MAX_HZ 300.0f
#endif
#ifndef TG_MACHETE_MIN_HZ
#define TG_MACHETE_MIN_HZ 5.0f
#endif
#ifndef TG_MACHETE_MAX_HZ
#define TG_MACHETE_MAX_HZ 10.0f
#endif

#ifndef TG_PRE_TOGGLE_DELAY_MS
#define TG_PRE_TOGGLE_DELAY_MS 100
#endif
#ifndef TG_ENABLE_INTERRUPT
#define TG_ENABLE_INTERRUPT true
#endif
#ifndef TG_MONITOR_DURATION_MS
#define TG_MONITOR_DURATION_MS 60000
#endif
#ifndef TG_VIBRATION_THRESHOLD
#define TG_VIBRATION_THRESHOLD 0.5f
#endif

#ifdef ENABLE_SD_LOGGER
#ifndef TG_BLOCKS_PER_BATCH
#define TG_BLOCKS_PER_BATCH 25
#endif
#ifndef TG_PASSIVE_MODE
#define TG_PASSIVE_MODE false
#endif
#ifndef TG_BATCH_SLEEP_MINUTES
#define TG_BATCH_SLEEP_MINUTES 20.0f
#endif
#ifndef TG_TOTAL_LOGGING_HOURS
#define TG_TOTAL_LOGGING_HOURS 24.0f
#endif
#ifndef TG_SIMULATED_BATCH_SECONDS
#define TG_SIMULATED_BATCH_SECONDS 1200
#endif
#ifndef TG_BATCH_DURATION_MS
#define TG_BATCH_DURATION_MS (TG_BLOCKS_PER_BATCH * TG_ACCEL_DURATION_SECONDS * 1000 + 200 * TG_BLOCKS_PER_BATCH)
#endif
#endif

/* ─────────────────────────────────────────────
   VEXT helpers (active-LOW on Heltec V3)
   ───────────────────────────────────────────── */
inline void TG_vextOn()
{
    pinMode(TG_VEXT_PIN, OUTPUT);
    digitalWrite(TG_VEXT_PIN, LOW);
}
inline void TG_vextOff()
{
    pinMode(TG_VEXT_PIN, OUTPUT);
    digitalWrite(TG_VEXT_PIN, HIGH);
}

/* ─────────────────────────────────────────────
   Legacy compatibility (lets old code compile)
   ───────────────────────────────────────────── */
#ifndef SDA_PIN
#define SDA_PIN TG_I2C_SDA
#endif
#ifndef SCL_PIN
#define SCL_PIN TG_I2C_SCL
#endif
#ifndef VEXT_CTRL_PIN
#define VEXT_CTRL_PIN TG_VEXT_PIN
#endif
#ifndef INTERRUPT_PIN
#define INTERRUPT_PIN TG_INT_PIN
#endif
