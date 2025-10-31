# ESP32-S3 Deep Sleep & Interrupt Wake-Up - Complete Porting Guide

**Purpose:** This document isolates the deep sleep and interrupt wake-up implementation from the vibration detection system, providing everything needed to port this power management architecture to another ESP32-S3 project.

**Target Hardware:** ESP32-S3 (tested on Heltec WiFi LoRa 32 V3)

**Key Features:**
- Deep sleep with dual wake sources (external interrupt + timer fallback)
- Proper peripheral power management (VEXT rail control)
- I2C sensor re-initialization after wake
- State persistence across sleep cycles (RTC memory, NVS, SD card)

---

## 1. Deep Sleep Configuration

### 1.1 Sleep Mode Type

**Mode:** `esp_deep_sleep_start()` - Full deep sleep (not light sleep)

**Power State:** All peripherals powered down, only RTC domain remains active

**Required Include:**
```cpp
#include <esp_sleep.h>
```

### 1.2 Sleep Initiation

All three firmware modes use the same basic pattern with slight variations:

#### Production Mode (`src/real_classifier.cpp:358-379`)
```cpp
void enterDeepSleep() {
     Serial.println("Preparing for deep sleep...");

     Serial.println("Setting GPIO7 correctly before sleep...");
    pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);
    LOG_DEBUG("GPIO7 State Before Sleep: ");  Serial.println(digitalRead(INTERRUPT_PIN));

     Serial.println("Disabling I2C...");
    Wire.end();

     Serial.println("Disabling sensors...");
    powerVEXT(false);
    delay(50);

    enableWakeInterrupt();

     Serial.println("Entering deep sleep...");
    delay(100);
    Serial.flush();
    delay(200);

    esp_deep_sleep_start();
}
```

#### SD Logger Mode (`src/data_logger_sd.cpp:228-234`)
```cpp
// PASSIVE_MODE: periodic wake-up for batch logging
esp_sleep_enable_timer_wakeup((uint64_t)(BATCH_SLEEP_MINUTES * 60.0 * 1e6));
esp_deep_sleep_start();

// Non-passive: permanent deep sleep after logging complete
esp_deep_sleep_start();
```

#### Test Mode (`src/sensor_test.cpp:225-246`)
```cpp
void enterDeepSleep() {
     Serial.println("Preparing for deep sleep...");

     Serial.println("Setting GPIO7 correctly before sleep...");
    pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);
    LOG_DEBUG("GPIO7 State Before Sleep: ");  Serial.println(digitalRead(INTERRUPT_PIN));

     Serial.println("Disabling I2C...");
    Wire.end();

     Serial.println("Disabling sensors...");
    powerVEXT(false);
    delay(50);

    enableWakeInterrupt();

     Serial.println("Entering deep sleep...");
    delay(100);
    Serial.flush();
    delay(200);

    esp_deep_sleep_start();
}
```

---

## 2. Interrupt Wake-Up Mechanism

### 2.1 GPIO Pin Configuration

**Wake Interrupt Pin:** GPIO 7 (defined in `boards/heltec_v3/variant.h:7`)

```cpp
#define INTERRUPT_PIN GPIO_NUM_7
```

**Required Include:**
```cpp
#include <driver/rtc_io.h>  // For RTC GPIO control
```

### 2.2 Dual Wake Source Configuration

The system uses **two wake sources** to ensure reliable operation:
1. **EXT0** - External interrupt on GPIO 7 (primary wake source)
2. **Timer** - Fallback periodic wake (ensures device doesn't sleep forever)

#### Production & Test Mode (`src/real_classifier.cpp:381-387`)
```cpp
void enableWakeInterrupt() {
    pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);
    rtc_gpio_pullup_dis((gpio_num_t)INTERRUPT_PIN);
    rtc_gpio_pulldown_en((gpio_num_t)INTERRUPT_PIN);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)INTERRUPT_PIN, 1);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}
```

**Constants:**
```cpp
#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP 86400   // 24 hours (real_classifier.cpp:23)
#define TIME_TO_SLEEP 259200  // 72 hours (sensor_test.cpp:63)
```

#### Test Mode with Error Checking (`src/sensor_test.cpp:184-200`)
```cpp
void enableWakeInterrupt() {
     Serial.println("Re-arming INT for next wake-up...");
    LOG_DEBUG("INTERRUPT_PIN = ");  Serial.println(INTERRUPT_PIN);
    pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);

    esp_err_t errI = esp_sleep_enable_ext0_wakeup(INTERRUPT_PIN, 1);
    rtc_gpio_pullup_dis(INTERRUPT_PIN);
    rtc_gpio_pulldown_en(INTERRUPT_PIN);
    esp_err_t errT = esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

    if (errI == ESP_OK && errT == ESP_OK) {
         Serial.println("All wake-up sources enabled successfully.");
    } else {
         Serial.println("ERROR: Failed to enable one or more wake-up sources.");
    }
}
```

#### Timer-Only Wake (SD Logger Batch Mode)
```cpp
// src/data_logger_sd.cpp:228
esp_sleep_enable_timer_wakeup((uint64_t)(BATCH_SLEEP_MINUTES * 60.0 * 1e6));
```

**Configuration Summary:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| **EXT0 GPIO** | GPIO_NUM_7 | External interrupt pin |
| **EXT0 Level** | 1 (HIGH) | Wakes on rising edge (LOW→HIGH) |
| **Pull Resistor** | PULLDOWN | Internal RTC pulldown enabled |
| **Timer (Production)** | 86400s (24h) | Fallback wake interval |
| **Timer (Test)** | 259200s (72h) | Extended fallback wake interval |
| **Timer (SD Logger)** | Configurable | Based on BATCH_SLEEP_MINUTES |

### 2.3 Disabling Wake Sources

**CRITICAL:** Wake sources persist across reboots. Must clear before reconfiguring.

```cpp
// src/real_classifier.cpp:389-391
void disableWakeInterrupt() {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
}

// src/sensor_test.cpp:171-181 (with error checking)
void disableWakeInterrupt() {
     Serial.println("Disabling all wake-up sources...");

    esp_err_t err = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    if (err == ESP_OK) {
         Serial.println("All wake-up sources disabled successfully.");
    } else {
         Serial.println("ERROR: Failed to disable one or more wake-up sources.");
    }
}
```

**Usage:** Called at boot in `sensor_test.cpp:95` to clear existing wake sources.

---

## 3. Pre-Sleep State Management

### 3.1 VEXT Power Control

**⚠️ CRITICAL: INVERTED LOGIC - `LOW = ON`, `HIGH = OFF`**

#### Pin Definition (`boards/heltec_v3/variant.h:6`)
```cpp
#define VEXT_CTRL_PIN GPIO_NUM_36
```

#### Power Control Function (`src/real_classifier.cpp:186-188`)
```cpp
void powerVEXT(bool state) {
    digitalWrite(VEXT_CTRL_PIN, state ? LOW : HIGH);
}
```

#### Power Down Before Sleep (`src/real_classifier.cpp:369-370`)
```cpp
powerVEXT(false);  // Sets VEXT_CTRL_PIN HIGH to turn OFF power rail
delay(50);         // Wait 50ms for sensors to power down completely
```

**What VEXT Powers:**
- MPU6050 (accelerometer/gyroscope)
- MAX17048 (battery fuel gauge)
- Any other peripherals on the VEXT rail

### 3.2 I2C Cleanup

**MUST** disable I2C before sleep to prevent current leakage.

```cpp
// src/real_classifier.cpp:366
Wire.end();  // Disable I2C peripheral
```

### 3.3 GPIO State Management

#### Interrupt Pin Preparation
```cpp
// src/real_classifier.cpp:362-363
pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);
LOG_DEBUG("GPIO7 State Before Sleep: ");  Serial.println(digitalRead(INTERRUPT_PIN));
```

#### RTC GPIO Configuration
```cpp
// src/real_classifier.cpp:383-384
rtc_gpio_pullup_dis((gpio_num_t)INTERRUPT_PIN);    // Disable pullup
rtc_gpio_pulldown_en((gpio_num_t)INTERRUPT_PIN);   // Enable pulldown
```

**Purpose:** Ensures RTC domain GPIO pull resistors remain active during deep sleep, allowing interrupt to wake the device.

### 3.4 Serial/UART Cleanup

**CRITICAL:** Flush serial buffer before sleep to prevent data loss.

```cpp
// src/real_classifier.cpp:376-377
Serial.flush();  // Ensure all serial data is transmitted
delay(200);      // Additional delay for UART hardware to complete
```

### 3.5 MPU6050 Sensor-Specific Handling

**⚠️ CRITICAL:** Prevent MPU6050 from entering its own sleep mode.

```cpp
// src/real_classifier.cpp:215
// src/sensor_test.cpp:274
mpu.setSleepEnabled(false); /*Prevents MPU to remain asleep after wakeup*/
delay(100);
```

**Why:** If the MPU6050 enters sleep mode, it won't respond to I2C commands after ESP32 wake-up.

### 3.6 Complete Pre-Sleep Sequence

**Timing is critical - follow this exact order:**

```
1. Log debug message
2. Configure interrupt pin (INPUT_PULLDOWN)
3. Disable I2C (Wire.end())
4. Power down VEXT (powerVEXT(false))
5. Wait 50ms
6. Enable wake interrupt sources
7. Log final message
8. Delay 100ms
9. Flush serial (Serial.flush())
10. Delay 200ms
11. Call esp_deep_sleep_start()
```

---

## 4. Post-Wake State Restoration

### 4.1 Wake Cause Detection

**Every `setup()` must check wake cause to determine action.**

```cpp
// src/real_classifier.cpp:54-56
esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
Serial.print("Wakeup cause: ");
Serial.println((int)wakeup_reason);
```

**Possible Wake Causes:**
- `ESP_SLEEP_WAKEUP_EXT0` - External interrupt (GPIO 7 triggered)
- `ESP_SLEEP_WAKEUP_TIMER` - Timer expired
- `ESP_SLEEP_WAKEUP_UNDEFINED` - Cold boot (power-on reset)

#### Production Mode Wake Handling (`src/real_classifier.cpp:83-94`)
```cpp
switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
        classifyAndStore();  // External interrupt triggered - do main work
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        statusOnlyMode();    // Timer wake-up - periodic health check
        break;
    default:
         Serial.println("[WARN] Unknown wake reason. Defaulting to statusOnlyMode.");
        statusOnlyMode();
        break;
}
delay(500);
enterDeepSleep();
```

#### Test Mode Wake Detection (`src/sensor_test.cpp:85-94`)
```cpp
esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
LOG_DEBUG("Wake-up reason: ");  Serial.println(wakeup_reason);

if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
     Serial.println("Woke up from GPIO7 (Mechanical Sensor Trigger)");
} else if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
     Serial.println("Woke up from Timer (72h)");
} else {
     Serial.println("Cold Boot (Power-on Reset)");
}
```

### 4.2 Power-Up Sequence

**Must follow exact timing and order for reliable sensor operation.**

#### Step 1: VEXT Power Rail Enable
```cpp
// src/real_classifier.cpp:103-105
pinMode(VEXT_CTRL_PIN, OUTPUT);
powerVEXT(true);  // Sets VEXT_CTRL_PIN LOW to turn ON power rail
delay(200);       // Wait 200ms for sensors to power up and stabilize
```

**⚠️ CRITICAL:** 200ms delay is necessary for sensor power stabilization.

#### Step 2: I2C Re-initialization
```cpp
// src/real_classifier.cpp:107-108
Wire.begin(SDA_PIN, SCL_PIN);
 Serial.println("I2C initialized.");
```

**Pin Definitions** (`boards/heltec_v3/variant.h:10-11`):
```cpp
#define SDA_PIN GPIO_NUM_41
#define SCL_PIN GPIO_NUM_42
```

### 4.3 Sensor Re-initialization

**Every sensor must be re-initialized after wake-up.**

#### MPU6050 Initialization (`src/real_classifier.cpp:206-230`)
```cpp
bool testMPU() {
     Serial.println("Initializing MPU6050...");

    mpu.initialize();
    if (!mpu.testConnection()) {
         Serial.println("ERROR: MPU6050 NOT detected!");
        return false;
    }

    mpu.setSleepEnabled(false); /*Prevents MPU to remain asleep after wakeup*/
    delay(100);

     Serial.println("Disabling Gyroscope...");
    mpu.setStandbyXGyroEnabled(true);
    mpu.setStandbyYGyroEnabled(true);
    mpu.setStandbyZGyroEnabled(true);
    delay(10);

    if (!verifyGyroDisabled()) {
         Serial.println("ERROR: Gyroscope not properly disabled! Aborting.");
        return false;
    }

     Serial.println("MPU6050 gyroscope disabled successfully.");
     Serial.println("MPU6050 initialized successfully.");
    return true;
}
```

**Key Points:**
- **MUST** call `mpu.setSleepEnabled(false)` to prevent sensor lockup
- Gyroscope disabled to save power (only accelerometer needed)
- Connection test ensures sensor is responding

#### MAX17048 Fuel Gauge Initialization (`src/real_classifier.cpp:241-288`)
```cpp
bool testMAX() {
    #ifdef USE_MAX1704X
       Serial.println("Initializing MAX17048...");
      if (!lipo.begin()) {
         Serial.println("[MAX1704x] begin() failed — sensor not detected.");
        return false;
      }
      delay(200);

      // Read version register
      Wire.beginTransmission(0x36);
      Wire.write(0x08);
      Wire.endTransmission(false);
      Wire.requestFrom(0x36, 2);
      uint16_t version = (Wire.read() << 8) | Wire.read();
      LOG_DEBUG("[MAX1704x] Version: 0x%04X\n", version);

      // Check sleep status
      Wire.beginTransmission(0x36);
      Wire.write(0x0C);
      Wire.endTransmission(false);
      Wire.requestFrom(0x36, 2);
      uint16_t config = (Wire.read() << 8) | Wire.read();
      bool sleeping = config & (1 << 7);
      LOG_DEBUG("[MAX1704x] CONFIG: 0x%04X — Sleeping: %s\n", config, sleeping ? "Yes" : "No");

      // Reset the chip
      Wire.beginTransmission(0x36);
      Wire.write(0xFE);
      Wire.write(0x00);
      Wire.write(0x54);
      Wire.endTransmission();
       Serial.println("[MAX1704x] Reset command sent.");

      delay(1000);
      if (!lipo.begin()) {
         Serial.println("[MAX1704x] Re-init after reset failed.");
        return false;
      }

      // Monitor initial SOC for stability
       Serial.println("[MAX1704x] Monitoring initial SOC...");
      for (int i = 0; i < 5; i++) {
        float voltage = lipo.getVoltage();
        float soc = lipo.getSOC();
        LOG_DEBUG("[MAX1704x] Voltage: %.2f V, SOC: %.2f %%\n", voltage, soc);
        delay(1000);
      }
      return true;
    #else
      return false;
    #endif
}
```

### 4.4 Complete Initialization Flow

```cpp
// src/real_classifier.cpp:50-73
void setup() {
    INIT_DEBUG_SERIAL();
     Serial.println("Booting real-time classifier...");

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    Serial.print("Wakeup cause: ");
    Serial.println((int)wakeup_reason);

    initSensors();  // Power up VEXT, I2C, MPU6050, MAX17048

    if (!LittleFS.begin(true)) {
         Serial.println("[ERROR] LittleFS mount failed!");
    } else {
         Serial.println("[✓] LittleFS mounted.");
    }

    // Mode-specific behavior follows...
}
```

---

## 5. Complete State Machine Flows

### 5.1 Production Mode (`real_classifier.cpp`)

#### Cold Boot (Power-On)
```
1. setup() called
2. Initialize serial (115200 baud)
3. Check wake cause → ESP_SLEEP_WAKEUP_UNDEFINED
4. initSensors()
   a. Configure VEXT_CTRL_PIN as OUTPUT
   b. powerVEXT(true) → VEXT_CTRL_PIN = LOW
   c. delay(200ms) - sensor stabilization
   d. Wire.begin(SDA_PIN, SCL_PIN)
   e. testMPU() → initialize, disable sleep, disable gyro
   f. testMAX() → initialize, reset, verify
5. Mount LittleFS
6. statusOnlyMode() (default action on cold boot)
   - Read temperature from MPU
   - Read voltage/SOC from MAX17048
   - Print status report
7. delay(500ms)
8. enterDeepSleep()
   a. pinMode(INTERRUPT_PIN, INPUT_PULLDOWN)
   b. Wire.end()
   c. powerVEXT(false) → VEXT_CTRL_PIN = HIGH
   d. delay(50ms)
   e. enableWakeInterrupt()
      - Configure RTC GPIO
      - Enable EXT0 on GPIO 7
      - Enable 24h timer
   f. delay(100ms)
   g. Serial.flush()
   h. delay(200ms)
   i. esp_deep_sleep_start()
```

#### Wake from EXT0 (GPIO 7 Interrupt)
```
1. setup() called
2. Check wake cause → ESP_SLEEP_WAKEUP_EXT0
3. initSensors() (full re-init as above)
4. Mount LittleFS
5. classifyAndStore()
   a. collectBlockOfData() (12 seconds @ 333Hz)
   b. classifyBufferedData()
   c. Read battery voltage/SOC
   d. Log to LittleFS
6. delay(500ms)
7. enterDeepSleep()
```

#### Wake from Timer (24h Fallback)
```
1. setup() called
2. Check wake cause → ESP_SLEEP_WAKEUP_TIMER
3. initSensors()
4. Mount LittleFS
5. statusOnlyMode()
   a. Read temperature from MPU
   b. Read voltage/SOC from MAX17048
   c. Print status report
6. delay(500ms)
7. enterDeepSleep()
```

### 5.2 SD Logger Mode (`data_logger_sd.cpp`)

#### Batch Logging Cycle (PASSIVE_MODE = true)
```
1. setup() called
2. Initialize VEXT, I2C, sensors
3. Initialize SD card (software SPI)
4. Load state from /log_state.txt
   - loggingStartTime
   - totalActiveCycles
5. Check if total logging window complete
   - If elapsed >= TOTAL_LOGGING_HOURS: permanent deep sleep
6. Loop BLOCKS_PER_BATCH times (25 blocks = ~5 minutes)
   a. readSensorsToFile() (12 seconds @ 333Hz)
   b. Save to /<FILE_PREFIX>_sd_<session><index>.csv
   c. Blink status LED
7. totalActiveCycles++
8. saveLogStateToSD()
9. esp_sleep_enable_timer_wakeup(BATCH_SLEEP_MINUTES * 60 * 1e6)  // 20 minutes
10. esp_deep_sleep_start()
```

**Timing:**
- Active logging: ~5 minutes (25 blocks × 12 seconds)
- Sleep: 20 minutes
- Repeat until 24 hours total elapsed

#### One-Shot Logging (PASSIVE_MODE = false)
```
1. setup() called
2. Initialize all peripherals as above
3. Loop NUM_BLOCKS times (25 blocks)
   a. readSensorsToFile()
4. Log complete
5. Permanent deep sleep (no timer wake)
```

### 5.3 Test Mode (`sensor_test.cpp`)

```
1. setup() called
2. Check wake cause
3. disableWakeInterrupt() (clear existing sources)
4. disablePeripherals() (Bluetooth, LEDs)
5. Initialize VEXT, I2C
6. testMPU()
   - Enable MPU6050 data-ready interrupt at 1kHz
   - checkMPU_DRDY_pin() (verify interrupt toggling)
7. testMAX()
   - validateMAX_ALERT_wiring() (verify alert pin)
   - testMAX_alert() (configure thresholds)
8. readSensorData() (single reading, print JSON)
9. enterDeepSleep()
```

---

## 6. State Persistence Across Sleep Cycles

### 6.1 RTC Memory (Survives Deep Sleep)

**Use `RTC_DATA_ATTR` for variables that must persist across deep sleep.**

```cpp
// src/data_logger_sd.cpp:53
RTC_DATA_ATTR int sessionFileIndex = 0;
```

**Characteristics:**
- Stored in RTC slow memory
- Survives deep sleep
- **Lost on power cycle or hard reset**
- Limited to 8KB total

### 6.2 NVS (Non-Volatile Storage)

**Use for data that must survive power loss.**

```cpp
// Required include
#include <Preferences.h>

// Read from NVS (src/data_logger_sd.cpp:192-194)
prefs.begin("datalogger", false);
sessionFileIndex = prefs.getInt("fileIndex", 0);
prefs.end();

// Write to NVS (src/data_logger_sd.cpp:299-303)
sessionFileIndex++;
prefs.begin("datalogger", false);
prefs.putInt("fileIndex", sessionFileIndex);
prefs.end();
```

**Characteristics:**
- Stored in flash memory
- Survives power loss and deep sleep
- Limited write cycles (~100,000)
- Slower than RTC memory

### 6.3 SD Card State File

**Use for complex state that needs to survive power loss.**

```cpp
// src/data_logger_sd.cpp:85-110
bool loadLogStateFromSD(uint64_t &logStart, int &cycles) {
  File file = SD.open(STATE_FILENAME, FILE_READ);
  if (!file) return false;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.startsWith("logStart=")) {
      logStart = strtoull(line.substring(9).c_str(), nullptr, 10);
    } else if (line.startsWith("activeCycles=")) {
      cycles = line.substring(13).toInt();
    }
  }

  file.close();
  return true;
}

void saveLogStateToSD(uint64_t logStart, int cycles) {
  File file = SD.open(STATE_FILENAME, FILE_WRITE);
  if (!file) return;

  file.printf("logStart=%llu\n", logStart);
  file.printf("activeCycles=%d\n", cycles);
  file.close();
}
```

**State File Location** (`boards/heltec_v3/variant.h:48`):
```cpp
#define STATE_FILENAME "/log_state.txt"
```

**Characteristics:**
- Unlimited size (limited by SD card)
- Survives power loss
- Slower than RTC/NVS
- Requires SD card present

---

## 7. Timing Constants & Critical Delays

### 7.1 Sleep Durations

| Constant | Value | Location | Purpose |
|----------|-------|----------|---------|
| `TIME_TO_SLEEP` | 86400s (24h) | real_classifier.cpp:23 | Timer fallback for production |
| `TIME_TO_SLEEP` | 259200s (72h) | sensor_test.cpp:63 | Timer fallback for testing |
| `BATCH_SLEEP_MINUTES` | 20.0 min | variant.h:74 | Sleep between SD logger batches |
| `TOTAL_LOGGING_HOURS` | 24.0 h | variant.h:75 | Total logging window |

### 7.2 Power Stabilization Delays

| Location | Duration | Purpose | Consequence if Omitted |
|----------|----------|---------|------------------------|
| real_classifier.cpp:105 | 200ms | VEXT power-up stabilization | Sensors not ready, I2C fails |
| real_classifier.cpp:370 | 50ms | VEXT power-down completion | Incomplete shutdown |
| real_classifier.cpp:215 | 100ms | MPU sleep disable settling | Sensor may not respond |
| real_classifier.cpp:376 | 100ms | Pre-serial flush | Incomplete UART transmission |
| real_classifier.cpp:377 | 200ms | Post-serial flush | Lost debug data |

### 7.3 Sensor-Specific Delays

```cpp
// MPU6050 initialization (real_classifier.cpp)
mpu.setSleepEnabled(false);
delay(100);  // CRITICAL: Allow sleep disable to take effect

mpu.setStandbyXGyroEnabled(true);
mpu.setStandbyYGyroEnabled(true);
mpu.setStandbyZGyroEnabled(true);
delay(10);   // Allow gyro standby to complete
```

---

## 8. Minimal Porting Template

**Complete working example - copy this to start your port:**

```cpp
#include <Arduino.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <Wire.h>

// ===== PIN DEFINITIONS =====
#define VEXT_CTRL_PIN GPIO_NUM_36  // CRITICAL: LOW = ON, HIGH = OFF
#define INTERRUPT_PIN GPIO_NUM_7
#define SDA_PIN GPIO_NUM_41
#define SCL_PIN GPIO_NUM_42

// ===== TIMING CONSTANTS =====
#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP 86400  // 24 hours in seconds

// ===== POWER MANAGEMENT =====
// CRITICAL: Inverted logic - LOW turns power ON
void powerVEXT(bool state) {
    digitalWrite(VEXT_CTRL_PIN, state ? LOW : HIGH);
}

// ===== WAKE INTERRUPT CONFIGURATION =====
void enableWakeInterrupt() {
    // Configure interrupt pin
    pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);

    // Configure RTC domain GPIO (required for deep sleep wake)
    rtc_gpio_pullup_dis((gpio_num_t)INTERRUPT_PIN);
    rtc_gpio_pulldown_en((gpio_num_t)INTERRUPT_PIN);

    // Enable dual wake sources
    esp_sleep_enable_ext0_wakeup((gpio_num_t)INTERRUPT_PIN, 1);  // Wake on HIGH
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void disableWakeInterrupt() {
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
}

// ===== DEEP SLEEP ENTRY =====
void enterDeepSleep() {
    Serial.println("Preparing for deep sleep...");

    // 1. Configure interrupt pin
    pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);

    // 2. Disable I2C
    Wire.end();

    // 3. Power down sensors
    powerVEXT(false);
    delay(50);  // Wait for sensors to power down

    // 4. Enable wake sources
    enableWakeInterrupt();

    // 5. Flush serial and sleep
    delay(100);
    Serial.flush();
    delay(200);

    esp_deep_sleep_start();  // Never returns
}

// ===== SENSOR INITIALIZATION =====
void initSensors() {
    Serial.println("Initializing sensors...");

    // 1. Configure and enable VEXT power rail
    pinMode(VEXT_CTRL_PIN, OUTPUT);
    powerVEXT(true);  // CRITICAL: true = LOW = ON
    delay(200);  // CRITICAL: Wait for sensor power stabilization

    // 2. Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    // 3. Initialize your sensors here
    // Example: MPU6050
    // mpu.initialize();
    // mpu.setSleepEnabled(false);  // CRITICAL: Prevent sensor lockup
    // delay(100);

    Serial.println("Sensors initialized.");
}

// ===== MAIN SETUP =====
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== ESP32-S3 Deep Sleep Example ===");

    // 1. Check wake cause
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    Serial.print("Wake cause: ");

    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.println("External interrupt (GPIO 7)");
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("Timer");
            break;
        default:
            Serial.println("Cold boot (power-on)");
            break;
    }

    // 2. Initialize all peripherals
    initSensors();

    // 3. Do your work based on wake cause
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.println("Performing interrupt-triggered work...");
            // Do your main work here
            break;

        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("Performing periodic check...");
            // Do periodic maintenance here
            break;

        default:
            Serial.println("Performing cold boot initialization...");
            // Do first-time setup here
            break;
    }

    // 4. Go back to sleep
    delay(500);
    enterDeepSleep();
}

// ===== MAIN LOOP =====
void loop() {
    // Never reached - device always sleeps in setup()
}
```

**How to Use This Template:**

1. Copy the entire template to your project
2. Add your sensor initialization code in `initSensors()`
3. Add your work logic in the `setup()` switch statement
4. Modify timing constants as needed
5. Flash and test

**Testing:**
- Cold boot: Should print "Cold boot", init sensors, sleep
- Interrupt wake: Connect GPIO 7 to 3.3V briefly - should wake and execute interrupt work
- Timer wake: Wait 24 hours (or reduce `TIME_TO_SLEEP`) - should wake and execute timer work

---

## 9. Critical Porting Notes

### ⚠️ VEXT Inverted Logic
**ALWAYS remember:** `LOW = ON`, `HIGH = OFF` for `VEXT_CTRL_PIN`

```cpp
powerVEXT(true);   // → digitalWrite(VEXT_CTRL_PIN, LOW)  → Power ON
powerVEXT(false);  // → digitalWrite(VEXT_CTRL_PIN, HIGH) → Power OFF
```

### ⚠️ MPU6050 Sleep Prevention
**MUST** call after every wake-up:
```cpp
mpu.setSleepEnabled(false);
delay(100);
```

Without this, the MPU6050 may enter its own sleep mode and stop responding to I2C.

### ⚠️ RTC GPIO Pull Configuration
**MUST** configure RTC domain pull resistors for wake interrupts:
```cpp
rtc_gpio_pullup_dis((gpio_num_t)INTERRUPT_PIN);
rtc_gpio_pulldown_en((gpio_num_t)INTERRUPT_PIN);
```

Without this, the interrupt pin won't wake the ESP32 from deep sleep.

### ⚠️ Serial Flushing
**MUST** flush serial before sleep to prevent data loss:
```cpp
delay(100);
Serial.flush();
delay(200);
esp_deep_sleep_start();
```

### ⚠️ Wake Source Re-arming
Wake sources are cleared on boot - **MUST** re-enable before each sleep cycle:
```cpp
// In enterDeepSleep():
enableWakeInterrupt();  // Must call every time
esp_deep_sleep_start();
```

### ⚠️ I2C Re-initialization
**MUST** call `Wire.begin()` after every wake since `Wire.end()` is called before sleep:
```cpp
// In enterDeepSleep():
Wire.end();

// In setup() after wake:
Wire.begin(SDA_PIN, SCL_PIN);
```

### ⚠️ Power Stabilization Delays
**NEVER** skip these delays:
- **200ms** after VEXT power-up (sensors need time to stabilize)
- **50ms** after VEXT power-down (complete shutdown)
- **200ms** after `Serial.flush()` (UART transmission complete)

### ⚠️ Timer Microsecond Conversion
**ALWAYS** use the `uS_TO_S_FACTOR`:
```cpp
#define uS_TO_S_FACTOR 1000000ULL
esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
```

The timer function expects **microseconds**, not seconds.

---

## 10. Troubleshooting Guide

### Problem: Device won't wake from interrupt

**Check:**
1. RTC GPIO pull resistors configured: `rtc_gpio_pulldown_en()`
2. Correct wake level: `esp_sleep_enable_ext0_wakeup(..., 1)` for HIGH trigger
3. Interrupt pin is actually toggling (use multimeter/oscilloscope)
4. VEXT is turned off before sleep (GPIO state may leak current)

### Problem: Device wakes immediately after sleep

**Check:**
1. Interrupt pin state before sleep (should be LOW for rising edge trigger)
2. RTC pull resistors (pulldown should hold pin LOW)
3. External circuit not holding pin HIGH

### Problem: Sensors don't respond after wake

**Check:**
1. VEXT power-up delay (200ms minimum)
2. `mpu.setSleepEnabled(false)` called after init
3. `Wire.begin()` called after wake
4. VEXT actually powered on (measure voltage on VEXT rail)

### Problem: Serial output garbled or incomplete

**Check:**
1. `Serial.flush()` called before sleep
2. 200ms delay after `Serial.flush()`
3. Baud rate correct (115200)
4. USB cable/connection stable

### Problem: High sleep current

**Check:**
1. VEXT powered off (`powerVEXT(false)` called)
2. `Wire.end()` called before sleep
3. Bluetooth disabled (if applicable)
4. LEDs turned off
5. No external pullups on powered-down sensors

### Problem: NVS/Preferences not persisting

**Check:**
1. `prefs.begin()` called before read/write
2. `prefs.end()` called after operations
3. Correct namespace name
4. Flash partition table includes NVS partition

---

## 11. Power Consumption Reference

**Typical Current Draw (Heltec V3):**

| State | Current | Notes |
|-------|---------|-------|
| Deep sleep (optimal) | ~5-10 µA | VEXT off, wake sources enabled |
| Deep sleep (poor) | ~500 µA - 2 mA | VEXT on or I2C not disabled |
| Active (no radio) | ~80-150 mA | Sensors on, processing |
| Active (WiFi/BLE) | ~200-400 mA | Radio transmitting |

**Power Optimization Checklist:**
- ✅ VEXT powered off before sleep
- ✅ `Wire.end()` called before sleep
- ✅ Bluetooth disabled (if not needed)
- ✅ WiFi disabled (if not needed)
- ✅ All LEDs turned off
- ✅ MPU6050 gyro disabled (use only accelerometer)

---

## 12. Hardware Requirements

**Minimum Hardware for Deep Sleep + Interrupt Wake:**

1. **ESP32-S3** (any variant)
2. **External wake source** connected to GPIO 7:
   - Mechanical switch (to 3.3V)
   - Comparator output (SW-420, TS881, etc.)
   - Other digital signal (3.3V logic level)
3. **VEXT-controlled power rail** (optional but recommended):
   - P-channel MOSFET or load switch
   - Controls power to sensors
   - Gate controlled by GPIO 36

**Optional Hardware:**
- MPU6050 accelerometer (I2C address 0x68)
- MAX17048 battery fuel gauge (I2C address 0x36)
- SD card module (SPI)
- Status LEDs

---

## 13. References

**Source Files (All references from commit `f0f421f`):**

- `src/real_classifier.cpp` - Production deep sleep implementation
- `src/data_logger_sd.cpp` - Batch logging with timer wake
- `src/sensor_test.cpp` - Test mode with extended sleep
- `boards/heltec_v3/variant.h` - Pin definitions and constants
- `include/DebugConfiguration.h` - Power save mode flags

**ESP32 Documentation:**
- [ESP32 Sleep Modes](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html)
- [RTC GPIO](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#rtc-gpio)

---

**END OF PORTING GUIDE**

---

## Quick Reference Card

**Copy this section for quick reference during porting:**

```cpp
// ===== CRITICAL PATTERNS =====

// VEXT Control (INVERTED LOGIC!)
pinMode(VEXT_CTRL_PIN, OUTPUT);
digitalWrite(VEXT_CTRL_PIN, LOW);   // ON
delay(200);                          // Stabilization
digitalWrite(VEXT_CTRL_PIN, HIGH);  // OFF
delay(50);                           // Shutdown

// I2C Lifecycle
Wire.begin(SDA_PIN, SCL_PIN);  // After wake
Wire.end();                     // Before sleep

// Wake Interrupt Setup
pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);
rtc_gpio_pullup_dis((gpio_num_t)INTERRUPT_PIN);
rtc_gpio_pulldown_en((gpio_num_t)INTERRUPT_PIN);
esp_sleep_enable_ext0_wakeup((gpio_num_t)INTERRUPT_PIN, 1);
esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

// Sleep Entry
delay(100);
Serial.flush();
delay(200);
esp_deep_sleep_start();

// MPU6050 Init
mpu.initialize();
mpu.setSleepEnabled(false);  // CRITICAL!
delay(100);

// Wake Detection
esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
// ESP_SLEEP_WAKEUP_EXT0 = interrupt
// ESP_SLEEP_WAKEUP_TIMER = timer
// ESP_SLEEP_WAKEUP_UNDEFINED = cold boot
```
