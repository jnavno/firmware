# ESP32 Deep Sleep & Wake-up Implementation Research

**Document Version:** 1.1
**Date:** 2025-10-30
**Meshtastic Firmware:** Master Branch (also applies to treeguard-module branch)

---

## Executive Summary

The Meshtastic firmware includes **comprehensive and mature support** for ESP32 deep sleep and interrupt-based wake-up functionality. The implementation provides multiple sleep states, GPIO interrupt wake-up, timer-based wake, and LoRa interrupt wake capabilities.

### Key Findings

**✅ Fully Supported Features:**
- Deep sleep with timer wake-up
- Light sleep with timer wake-up
- GPIO interrupt wake-up (buttons, external signals)
- LoRa radio interrupt wake-up
- PMU (Power Management Unit) interrupt wake-up
- Touch screen interrupt wake-up
- Multi-level power state management via PowerFSM

**⚠️ Current Limitations:**
- No EXT0 wake-up implementation (single GPIO with level detection)
- No ULP (Ultra-Low Power) coprocessor usage
- No touchpad wake (though pins support it)
- Some platform inconsistencies (NRF52 deep sleep disabled)
- WiFi-enabled devices don't enter automatic power-saving modes

**Power Management:** The system uses a sophisticated Finite State Machine (PowerFSM) with 10 distinct power states, enabling fine-grained control over power consumption based on device role, battery status, and user configuration.

---

## Table of Contents

1. [Core Implementation Files](#core-implementation-files)
2. [Deep Sleep Implementation](#deep-sleep-implementation)
3. [Light Sleep Implementation](#light-sleep-implementation)
4. [Power Management Architecture](#power-management-architecture)
5. [Wake-up Sources](#wake-up-sources)
6. [Platform-Specific ESP32 Code](#platform-specific-esp32-code)
7. [Configuration and Variants](#configuration-and-variants)
8. [Current Limitations and Gaps](#current-limitations-and-gaps)
9. [Future Development Opportunities](#future-development-opportunities)
10. [Developer Guide](#developer-guide)
11. [References](#references)

---

## Core Implementation Files

### Primary Sleep Files

| File | Lines | Purpose |
|------|-------|---------|
| `src/sleep.cpp` | 546 | Main sleep implementation for all platforms |
| `src/sleep.h` | 55 | Sleep API headers and function declarations |
| `src/platform/esp32/main-esp32.cpp` | 266 | ESP32-specific deep sleep implementation |
| `src/PowerFSM.cpp` | 632 | Power state machine controlling sleep transitions |
| `src/PowerFSM.h` | 81 | Power state definitions and events |
| `src/Power.cpp` | 1058 | Battery monitoring and power management |

### Related Files

- `src/ButtonThread.cpp` - Button interrupt handling and sleep coordination
- `src/main.cpp` - Boot/wake detection integration
- `src/mesh/RadioInterface.cpp` - LoRa interrupt wake coordination
- `variants/*/variant.h` - Hardware-specific pin and wake configurations

---

## Deep Sleep Implementation

### Overview

Deep sleep (also called "Super Deep Sleep" or SDS in the code) is the lowest power state where:
- CPU is powered down
- RAM contents are lost
- RTC domain remains powered
- Wake-up sources: GPIO interrupts, timer
- Typical current: < 1mA (goal stated in PowerFSM.cpp:61)

### Entry Function: `doDeepSleep()`

**Location:** `src/sleep.cpp:198-345`

**Function Signature:**
```cpp
void doDeepSleep(uint64_t msecToWake, bool skipPreflight = false)
```

**Parameters:**
- `msecToWake`: Duration to sleep in milliseconds before timer wake
- `skipPreflight`: Skip observer notification (used for low battery shutdown)

**Execution Sequence:**

1. **Pre-sleep Notifications** (lines 198-220)
   ```cpp
   if (!skipPreflight) {
       waitEnterSleep(true); // Wait for subsystems to allow sleep
   }
   notifyDeepSleep.notifyObservers(NULL); // Final shutdown notifications
   ```

2. **Display Shutdown** (line 225)
   ```cpp
   if (screen)
       screen->doDeepSleep();
   ```

3. **Persistence** (line 228)
   ```cpp
   nodeDB->saveToDisk(); // Save database before power loss
   ```

4. **Peripheral Power-down** (lines 231-266)
   - GPS power off via GPIO or PMU
   - External peripherals powered down
   - LED power off
   - Radio CS pin held high to prevent leakage

5. **GPIO Hold Configuration** (lines 291-297, ESP32 only)
   ```cpp
   #if SOC_RTCIO_HOLD_SUPPORTED
   gpio_hold_en((gpio_num_t)LORA_CS);
   gpio_deep_sleep_hold_en();
   #endif
   ```

6. **PMU Configuration** (lines 300-333, if PMU present)
   - Configures power rails for shutdown
   - Different paths for AXP192 vs AXP2101

7. **Platform-Specific Deep Sleep** (line 335)
   ```cpp
   #ifdef ARCH_ESP32
   cpuDeepSleep(msecToWake);
   #elif defined(ARCH_NRF52)
   // ... NRF52 implementation
   #endif
   ```

### ESP32-Specific: `cpuDeepSleep()`

**Location:** `src/platform/esp32/main-esp32.cpp:197-265`

**Function Signature:**
```cpp
esp_sleep_wakeup_cause_t cpuDeepSleep(uint64_t msecToWake)
```

**Implementation Details:**

1. **RTC GPIO Isolation** (lines 214-227)
   ```cpp
   #if SOC_RTCIO_HOLD_SUPPORTED
   // Isolate unused RTC GPIOs to prevent current leakage
   for (gpio_num_t gpio_num = GPIO_NUM_0; gpio_num < GPIO_NUM_MAX; gpio_num++) {
       if (GPIO_IS_VALID_GPIO(gpio_num) && rtc_gpio_is_valid_gpio(gpio_num)) {
           bool is_held = gpio_hal_get_pin_hold(&GPIO_HAL, gpio_num);
           if (!is_held) {
               rtc_gpio_isolate(gpio_num);
           }
       }
   }
   #endif
   ```

2. **Button Wake Configuration** (lines 231-258)
   ```cpp
   uint32_t button_gpio_pin = ...;
   uint64_t gpioMask = (1ULL << button_gpio_pin);

   #ifdef BUTTON_NEED_PULLUP
   gpio_pullup_en((gpio_num_t)button_gpio_pin);
   #endif

   // Platform-specific wake type
   #ifdef ESP32S3_WAKE_TYPE
   esp_sleep_enable_ext1_wakeup_io(gpioMask, ESP32S3_WAKE_TYPE);
   #elif CONFIG_IDF_TARGET_ESP32
   esp_sleep_enable_ext1_wakeup(gpioMask, ESP_EXT1_WAKEUP_ALL_LOW);
   #else
   esp_sleep_enable_ext1_wakeup_io(gpioMask, ESP_EXT1_WAKEUP_ANY_LOW);
   #endif
   ```

3. **RTC Domain Configuration** (line 261)
   ```cpp
   esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
   ```
   - Keeps RTC peripherals powered for GPIO wake-up

4. **Timer Wake Configuration** (line 263)
   ```cpp
   esp_sleep_enable_timer_wakeup(msecToWake * 1000ULL); // Convert ms to μs
   ```

5. **Enter Deep Sleep** (line 264)
   ```cpp
   esp_deep_sleep_start(); // Does not return
   ```

### Boot/Wake Detection: `initDeepSleep()`

**Location:** `src/sleep.cpp:99-166`

Called during boot to determine if waking from sleep or cold boot.

**Wake Cause Detection:**
```cpp
esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT0:
        LOG_INFO("Wake from ext0 RTC_IO interrupt\n");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        LOG_INFO("Wake from ext1 RTC_CNTL interrupt\n");
        // Note: Could read which GPIO via esp_sleep_get_ext1_wakeup_status()
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        LOG_INFO("Wake from timer\n");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        LOG_INFO("Wake from touchpad\n");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        LOG_INFO("Wake from ULP program\n");
        break;
    default:
        LOG_INFO("Wake from reset or power on\n");
        break;
}
```

**GPIO Hold Release** (lines 150-163):
```cpp
#if SOC_RTCIO_HOLD_SUPPORTED
if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis((gpio_num_t)LORA_CS);
}
#endif
```

**Global State:**
- Stores wake cause in `wakeCause` global variable
- Used by Screen, PowerFSM, and main.cpp for wake reason handling

---

## Light Sleep Implementation

### Overview

Light sleep is a power-saving mode where:
- CPU is suspended
- RAM contents are preserved
- Wake-up is faster than deep sleep
- More wake-up sources available
- Typical current: 1-5mA (depending on peripherals)

### Entry Function: `doLightSleep()`

**Location:** `src/sleep.cpp:353-470`

**Function Signature:**
```cpp
esp_sleep_wakeup_cause_t doLightSleep(uint64_t sleepMsec)
```

**Returns:** Wake cause enum for handling by caller (typically PowerFSM)

**Execution Sequence:**

1. **Special Case Handling** (lines 359-361)
   ```cpp
   #ifdef SENSECAP_INDICATOR
   return ESP_SLEEP_WAKEUP_TIMER; // Extended IO pin issue workaround
   #endif
   ```

2. **Pre-sleep Notifications** (lines 363-368)
   ```cpp
   waitEnterSleep(false); // Allow observer veto
   notifyLightSleep.notifyObservers(NULL); // Notify light sleep entry

   // Required by ESP-IDF docs
   #if !MESHTASTIC_EXCLUDE_BLUETOOTH
   esp_bluedroid_disable(); // or nimble equivalent
   #endif
   esp_wifi_stop();
   ```

3. **RTC Configuration** (line 371)
   ```cpp
   esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
   ```

4. **Wake Source Configuration** (lines 396-420)

   **Button Wake:**
   ```cpp
   gpio_wakeup_enable((gpio_num_t)BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
   esp_sleep_enable_gpio_wakeup();
   ```

   **Encoder Button:**
   ```cpp
   #ifdef INPUTDRIVER_ENCODER_BTN
   gpio_wakeup_enable((gpio_num_t)INPUTDRIVER_ENCODER_BTN, GPIO_INTR_LOW_LEVEL);
   #endif
   ```

   **Touch Screen:**
   ```cpp
   #ifdef WAKE_ON_TOUCH
   gpio_wakeup_enable((gpio_num_t)SCREEN_TOUCH_INT, GPIO_INTR_LOW_LEVEL);
   #endif
   ```

   **LoRa Interrupt:**
   ```cpp
   enableLoraInterrupt();
   ```

   **PMU IRQ:**
   ```cpp
   #if HAS_PMU
   if (pmu_found)
       gpio_wakeup_enable((gpio_num_t)PMU_IRQ, GPIO_INTR_LOW_LEVEL);
   #endif
   ```

   **Timer:**
   ```cpp
   esp_sleep_enable_timer_wakeup(sleepMsec * 1000ULL);
   ```

5. **Enter Light Sleep** (line 433)
   ```cpp
   esp_light_sleep_start();
   ```

6. **Wake-up Processing** (lines 436-469)
   ```cpp
   // Disable GPIO wakeups
   esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
   gpio_wakeup_disable((gpio_num_t)BUTTON_PIN);
   // ... other GPIOs

   // Notify wake
   esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
   notifyLightSleepEnd.notifyObserversWithArg(cause);

   // Log wake reason
   LOG_DEBUG("Woke from light sleep: %d\n", cause);

   return cause;
   ```

### LoRa Interrupt Wake: `enableLoraInterrupt()`

**Location:** `src/sleep.cpp:511-544`

**Platform Detection:**
```cpp
#if SOC_PM_SUPPORT_EXT_WAKEUP
// Newer ESP32 variants (S2, S3, C3, etc.)
gpio_pulldown_en((gpio_num_t)LORA_DIO1);
gpio_pullup_en((gpio_num_t)LORA_RESET);
gpio_pullup_en((gpio_num_t)LORA_CS);
gpio_wakeup_enable((gpio_num_t)LORA_DIO1, GPIO_INTR_HIGH_LEVEL);
#else
// Original ESP32
#if defined(LORA_DIO1) // SX126x or SX128x
gpio_wakeup_enable((gpio_num_t)LORA_DIO1, GPIO_INTR_HIGH_LEVEL);
#elif defined(RF95_IRQ) // RF95
gpio_wakeup_enable((gpio_num_t)RF95_IRQ, GPIO_INTR_HIGH_LEVEL);
#endif
#endif
```

**Conditional Enablement** (lines 505-509):
```cpp
bool shouldLoraWake() {
    return msecToWake < portMAX_DELAY &&
           (config.device.role == meshtastic_Config_DeviceConfig_Role_ROUTER ||
            config.device.role == meshtastic_Config_DeviceConfig_Role_REPEATER);
}
```

Only ROUTER and REPEATER roles wake on LoRa packets to maintain mesh connectivity.

---

## Power Management Architecture

### PowerFSM State Machine

**Location:** `src/PowerFSM.cpp`

The PowerFSM (Finite State Machine) orchestrates all power state transitions based on events, timers, and configuration.

### Power States

| State | Description | Screen | Bluetooth | CPU | Radio | Current (est) |
|-------|-------------|--------|-----------|-----|-------|---------------|
| **BOOT** | Initial boot state | Initializing | Off | Full | Init | ~150mA |
| **ON** | Normal operation | On | On | 80MHz | Active | ~80-120mA |
| **POWER** | Externally powered | On | On | 80MHz | Active | ~80-120mA |
| **DARK** | Screen off | Off | On | 80MHz | Active | ~60-80mA |
| **SERIAL** | Serial API active | Off | Off | 80MHz | Active | ~40-60mA |
| **NB** (No Bluetooth) | BT off, preparing sleep | Off | Off | 80MHz | Active | ~40-60mA |
| **LS** (Light Sleep) | CPU suspended | Off | Off | Suspended | Wake-on-IRQ | ~1-5mA |
| **SDS** (Super Deep Sleep) | Deep sleep | Off | Off | Off | Off | < 1mA |
| **LowBattSDS** | Emergency low battery sleep | Off | Off | Off | Off | < 1mA |
| **SHUTDOWN** | Complete shutdown | Off | Off | Off | Off | ~0mA |

### State Transitions

**Normal Operation Flow:**
```
BOOT → (check power) → POWER or ON
  ↓
ON → (screen_on_secs timeout) → DARK
  ↓
DARK → (wait_bluetooth_secs timeout) → NB [if power-saving enabled]
  ↓
NB → (min_wake_secs timeout) → LS
  ↓
LS → (timer or interrupt wake) → NB or DARK
  ↓
[repeat LS ↔ NB cycle]
```

**Router/Power-Saving Flow:**
```
BOOT → ON → DARK → LS (skip NB if no Bluetooth needed)
  ↓
LS → (wake timer) → LS (immediate return to sleep if no activity)
```

**Low Battery Flow:**
```
Any State → (EVENT_LOW_BATTERY) → LowBattSDS
```

### Power State Implementations

#### BOOT State (lines 252-255)
```cpp
fsmTransition(&stateBOOT, EVENT_BOOT, 0, &statePOWER, "Set power on");
```
- Entry point after device boot or reset
- Immediate transition to POWER or ON based on external power detection

#### ON State (lines 232-245)
```cpp
stateON.timeoutMs = getConfiguredOrDefaultMs(config.display.screen_on_secs);
stateON.transitionTo(EVENT_TIMEOUT) = &stateDARK;
stateON.transitionTo(EVENT_PRESS) = &stateON; // Restart timer
stateON.transitionTo(EVENT_BLUETOOTH_PAIR) = &stateON;
```
- Normal operation with screen on
- Timeout → DARK after `screen_on_secs` (default: 10 minutes)
- Button press restarts timer
- All Bluetooth events restart timer

#### POWER State (lines 193-230)
```cpp
statePOWER.enter = []() {
    // Externally powered - don't do any power saving
    LOG_DEBUG("State transition: POWER\n");
};
```
- Similar to ON but detected external power
- Bypasses some power-saving logic
- Transitions like ON state

#### DARK State (lines 172-176)
```cpp
stateDARK.timeoutMs = getConfiguredOrDefaultMs(config.power.wait_bluetooth_secs);
stateDARK.transitionTo(EVENT_TIMEOUT) = shouldLightSleep() ? &stateLS : nullptr;
```
- Screen off, Bluetooth still active
- Timeout → LS if power-saving enabled
- Button press → ON (via PowerFSM event handling)

#### NB (No Bluetooth) State (lines 160-170, ESP32 only)
```cpp
stateNB.timeoutMs = getConfiguredOrDefaultMs(config.power.min_wake_secs);
stateNB.transitionTo(EVENT_TIMEOUT) = &stateLS;
```
- Bluetooth disabled
- Minimum wake time before returning to light sleep
- Only entered from LS wake on ESP32

#### LS (Light Sleep) State (lines 82-158)
```cpp
stateLS.enter = []() {
    LOG_INFO("Entering light sleep\n");
    // Power-saving state with CPU suspended
};

// On enter:
uint32_t sleepTime = getSleepTime();
esp_sleep_wakeup_cause_t cause = doLightSleep(sleepTime);

// Wake handling:
switch (cause) {
    case ESP_SLEEP_WAKEUP_TIMER:
        // Check if should wake or return to sleep
        break;
    case ESP_SLEEP_WAKEUP_GPIO:
        // Check button state
        if (digitalRead(BUTTON_PIN) == activeState) {
            powerFSM.trigger(EVENT_PRESS);
        }
        break;
    // ... other causes
}
```
- CPU in light sleep
- Wakes on timer or interrupts
- Intelligent wake handling (may return to sleep immediately)
- Blinks LED on timer wake (if configured)

#### SDS (Super Deep Sleep) State (lines 58-63)
```cpp
stateSDS.enter = []() {
    LOG_INFO("Entering deep sleep for %lu seconds\n",
             config.power.sds_secs);
    doDeepSleep(config.power.sds_secs * 1000ULL);
};
```
- Deep sleep for configured duration
- Entered via explicit transition or low battery
- Device resets on wake (boots from scratch)

#### LowBattSDS State (lines 65-69)
```cpp
stateLowBattSDS.enter = []() {
    LOG_INFO("Low battery, entering deep sleep\n");
    doDeepSleep(UINT32_MAX, true); // Skip preflight, sleep indefinitely
};
```
- Emergency deep sleep when battery critically low
- Skips NodeDB save to prevent corruption
- Sleeps indefinitely until external power or button press

### Power State Events

**Defined in:** `src/PowerFSM.h:18-42`

```cpp
#define EVENT_PRESS 1              // Button press
#define EVENT_WAKE_TIMER 2         // Wake timer expired
#define EVENT_PACKET_FOR_PHONE 4   // Packet received for phone
#define EVENT_RECEIVED_MSG 5       // Message received
#define EVENT_BLUETOOTH_PAIR 7     // Bluetooth pairing
#define EVENT_NODEDB_UPDATED 8     // NodeDB changed
#define EVENT_LOW_BATTERY 10       // Battery critically low
#define EVENT_SERIAL_CONNECTED 11  // Serial API connected
#define EVENT_SERIAL_DISCONNECTED 12
#define EVENT_POWER_CONNECTED 13   // External power connected
#define EVENT_POWER_DISCONNECTED 14
#define EVENT_SHUTDOWN 16          // Force shutdown
#define EVENT_INPUT 17             // Input device activity
```

**Event Triggering Examples:**
```cpp
// Button press (ButtonThread.cpp)
powerFSM.trigger(EVENT_PRESS);

// Low battery (Power.cpp)
if (batteryLevel < 5 && !isPowered) {
    powerFSM.trigger(EVENT_LOW_BATTERY);
}

// Message received (MeshService.cpp)
powerFSM.trigger(EVENT_RECEIVED_MSG);
```

### Configuration Parameters

**Location:** `src/mesh/Default.h:131-160`

```cpp
// Normal device defaults
default_wait_bluetooth_secs = 60        // Time in DARK before → LS
default_sds_secs = UINT32_MAX           // Deep sleep duration (disabled)
default_ls_secs = 300                   // Light sleep duration (5 min)
default_min_wake_secs = 10              // Minimum wake time in NB
default_screen_on_secs = 600            // Screen on time (10 min)

// Router defaults (via IF_ROUTER macro)
default_wait_bluetooth_secs = 1         // Fast transition to sleep
default_sds_secs = ONE_DAY              // Deep sleep 24 hours
default_ls_secs = ONE_DAY               // Light sleep 24 hours
default_screen_on_secs = 1              // Screen on 1 second
```

**User Configuration Fields:**
```cpp
config.power.ls_secs                    // Light sleep duration
config.power.sds_secs                   // Deep sleep duration
config.power.min_wake_secs              // Minimum wake time
config.power.wait_bluetooth_secs        // DARK state timeout
config.power.is_power_saving            // Enable aggressive power saving
config.power.on_battery_shutdown_after_secs  // Auto-shutdown timeout
config.display.screen_on_secs           // Screen on duration
```

### Power-Saving Enablement Logic

**Location:** `src/PowerFSM.cpp:384-405`

```cpp
bool shouldEnterPowerSaving() {
    // Always power-save if router
    if (config.device.role == meshtastic_Config_DeviceConfig_Role_ROUTER)
        return true;

    // Explicitly enabled
    if (config.power.is_power_saving)
        return true;

    // Disable for WiFi (unstable with sleep)
    if (isWifiAvailable())
        return false;

    // Disable for trackers (they control their own timing)
    if (config.device.role == meshtastic_Config_DeviceConfig_Role_TRACKER ||
        config.device.role == meshtastic_Config_DeviceConfig_Role_TAK_TRACKER ||
        config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR)
        return false;

    return false; // Default: no power-saving
}
```

---

## Wake-up Sources

### GPIO Interrupts

#### Deep Sleep: EXT1 Wake-up

**Implementation:** `src/platform/esp32/main-esp32.cpp:231-258`

**Mechanism:**
- Uses ESP-IDF's `esp_sleep_enable_ext1_wakeup()` or `esp_sleep_enable_ext1_wakeup_io()`
- Creates GPIO bitmask for button pin
- Wake logic varies by platform:
  - **ESP32**: `ESP_EXT1_WAKEUP_ALL_LOW` (all pins must be low)
  - **ESP32S2/C3/S3**: `ESP_EXT1_WAKEUP_ANY_LOW` (any pin low)
  - **Custom per variant**: `ESP32S3_WAKE_TYPE` define overrides

**Supported RTC GPIOs (ESP32):**
- GPIO 0, 2, 4, 12, 13, 14, 15, 25, 26, 27, 32-39
- Only these pins can wake from deep sleep

**Example Configuration:**
```cpp
// Heltec Capsule Sensor V3 (variant.h:53)
#define ESP32S3_WAKE_TYPE ESP_EXT1_WAKEUP_ANY_HIGH
```

**Code:**
```cpp
uint32_t button_gpio_pin = ... // From config or BUTTON_PIN
uint64_t gpioMask = (1ULL << button_gpio_pin);

#ifdef BUTTON_NEED_PULLUP
gpio_pullup_en((gpio_num_t)button_gpio_pin);
#endif

#ifdef ESP32S3_WAKE_TYPE
esp_sleep_enable_ext1_wakeup_io(gpioMask, ESP32S3_WAKE_TYPE);
#elif CONFIG_IDF_TARGET_ESP32
esp_sleep_enable_ext1_wakeup(gpioMask, ESP_EXT1_WAKEUP_ALL_LOW);
#else
esp_sleep_enable_ext1_wakeup_io(gpioMask, ESP_EXT1_WAKEUP_ANY_LOW);
#endif
```

**Wake Detection:**
```cpp
// On boot (sleep.cpp:110-111)
case ESP_SLEEP_WAKEUP_EXT1:
    LOG_INFO("Wake from ext1 RTC_CNTL interrupt\n");
    // Can read which GPIO via esp_sleep_get_ext1_wakeup_status()
```

#### Light Sleep: GPIO Wakeup

**Implementation:** `src/sleep.cpp:396-420`

**Mechanism:**
- Uses `gpio_wakeup_enable()` + `esp_sleep_enable_gpio_wakeup()`
- Works on any GPIO pin, not just RTC pins
- Supports level-triggered interrupts

**Supported GPIOs:**
- Button: `BUTTON_PIN` or `config.device.button_gpio`
- Encoder: `INPUTDRIVER_ENCODER_BTN`
- Touch: `SCREEN_TOUCH_INT` (if `WAKE_ON_TOUCH` defined)
- LoRa: `LORA_DIO1` or `RF95_IRQ`
- PMU: `PMU_IRQ`

**Code:**
```cpp
// Button wake
gpio_wakeup_enable((gpio_num_t)BUTTON_PIN, GPIO_INTR_LOW_LEVEL);

// Encoder button
#ifdef INPUTDRIVER_ENCODER_BTN
gpio_wakeup_enable((gpio_num_t)INPUTDRIVER_ENCODER_BTN, GPIO_INTR_LOW_LEVEL);
#endif

// Touch screen
#ifdef WAKE_ON_TOUCH
gpio_wakeup_enable((gpio_num_t)SCREEN_TOUCH_INT, GPIO_INTR_LOW_LEVEL);
#endif

// Enable GPIO wakeup globally
esp_sleep_enable_gpio_wakeup();
```

**Wake Handling (PowerFSM.cpp:128-140):**
```cpp
case ESP_SLEEP_WAKEUP_GPIO:
    // Check actual button state to filter spurious wakes
    if (digitalRead(BUTTON_PIN) == BUTTON_ACTIVE_STATE) {
        powerFSM.trigger(EVENT_PRESS);
    } else {
        // Spurious wake, treat as timer wake
        powerFSM.trigger(EVENT_WAKE_TIMER);
    }
```

**Cleanup on Wake (sleep.cpp:436-454):**
```cpp
esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
gpio_wakeup_disable((gpio_num_t)BUTTON_PIN);
#ifdef INPUTDRIVER_ENCODER_BTN
gpio_wakeup_disable((gpio_num_t)INPUTDRIVER_ENCODER_BTN);
#endif
// ... disable other GPIOs
```

### Timer Wake-up

**Deep Sleep Timer:**
```cpp
// main-esp32.cpp:263
esp_sleep_enable_timer_wakeup(msecToWake * 1000ULL); // μs
```

**Light Sleep Timer:**
```cpp
// sleep.cpp:420
esp_sleep_enable_timer_wakeup(sleepMsec * 1000ULL); // μs
```

**Timer Wake Handling:**
- Deep sleep: Device resets, appears as normal boot
- Light sleep: Returns from `doLightSleep()` with `ESP_SLEEP_WAKEUP_TIMER`

**Dynamic Sleep Time Calculation (PowerFSM.cpp:407-446):**
```cpp
uint32_t getSleepTime() {
    uint32_t sleepTime = config.power.ls_secs * 1000;

    // Router: Adjust for mesh intervals
    if (role == ROUTER || role == REPEATER) {
        uint32_t meshInterval = getMeshInterval();
        if (meshInterval > 0) {
            sleepTime = meshInterval + 1000; // Wake 1s after expected packet
        }
    }

    // Position module: Align with GPS intervals
    if (positionModule) {
        uint32_t gpsInterval = positionModule->getNextUpdateMs();
        sleepTime = min(sleepTime, gpsInterval);
    }

    // Telemetry module: Align with sensor intervals
    if (telemetryModule) {
        uint32_t telemetryInterval = telemetryModule->getNextUpdateMs();
        sleepTime = min(sleepTime, telemetryInterval);
    }

    return sleepTime;
}
```

### LoRa Interrupt Wake

**Function:** `enableLoraInterrupt()` in `src/sleep.cpp:511-544`

**Conditional Enablement:**
```cpp
bool shouldLoraWake() {
    return msecToWake < portMAX_DELAY &&
           (config.device.role == meshtastic_Config_DeviceConfig_Role_ROUTER ||
            config.device.role == meshtastic_Config_DeviceConfig_Role_REPEATER);
}
```

**Platform-Specific Implementation:**

**Newer ESP32 variants (S2, S3, C3, C6):**
```cpp
#if SOC_PM_SUPPORT_EXT_WAKEUP
gpio_pulldown_en((gpio_num_t)LORA_DIO1);
gpio_pullup_en((gpio_num_t)LORA_RESET);
gpio_pullup_en((gpio_num_t)LORA_CS);
gpio_wakeup_enable((gpio_num_t)LORA_DIO1, GPIO_INTR_HIGH_LEVEL);
#endif
```

**Original ESP32:**
```cpp
#if defined(LORA_DIO1) // SX126x or SX128x
gpio_wakeup_enable((gpio_num_t)LORA_DIO1, GPIO_INTR_HIGH_LEVEL);
#elif defined(RF95_IRQ) // RF95
gpio_wakeup_enable((gpio_num_t)RF95_IRQ, GPIO_INTR_HIGH_LEVEL);
#endif
```

**Radio Types:**
- **SX126x/SX128x**: Use `LORA_DIO1` pin
- **RF95**: Use `RF95_IRQ` pin
- **LR11xx**: Use `LORA_DIO1` pin

**Wake Logic:**
- Triggered when LoRa chip asserts interrupt (packet received)
- Only enabled for ROUTER/REPEATER roles
- Allows maintaining mesh connectivity while sleeping

### Button Interrupt Coordination

**Normal Operation Interrupts (ButtonThread.cpp:337-371):**
```cpp
void attachButtonInterrupts() {
    attachInterrupt(
        BUTTON_PIN,
        handleButtonInterrupt,
        BUTTON_ACTIVE_STATE ? RISING : FALLING
    );
}

IRAM_ATTR void handleButtonInterrupt() {
    // Signal main thread
    mainDelay.interruptFromISR();
}
```

**Sleep Handoff (ButtonThread.cpp:400-418):**
```cpp
// Observer: notifyLightSleep
void beforeLightSleep() {
    // Detach our interrupts - sleep will configure its own
    detachButtonInterrupts();
}

// Observer: notifyLightSleepEnd
void afterLightSleep(esp_sleep_wakeup_cause_t cause) {
    // Reattach interrupts for normal operation
    attachButtonInterrupts();
}
```

**Why the Handoff?**
- Normal operation: Fast interrupt response via `attachInterrupt()`
- Light sleep: Different mechanism via `gpio_wakeup_enable()`
- Cannot have both active simultaneously
- Observable pattern ensures clean transition

### PMU (Power Management Unit) Interrupt

**Supported PMUs:**
- AXP192 (T-Beam v1.0, v1.1)
- AXP2101 (T-Beam v1.2+, some Heltec boards)

**Wake Configuration (sleep.cpp:410-414):**
```cpp
#if HAS_PMU
if (pmu_found) {
    gpio_wakeup_enable((gpio_num_t)PMU_IRQ, GPIO_INTR_LOW_LEVEL);
}
#endif
```

**Wake Reasons:**
- Battery voltage change
- Charging status change
- Power button press (if wired through PMU)
- Over-temperature alert
- Low battery alert

**Implementation:** `src/Power.cpp:931-1057`

### Wake Source Priority

When multiple wake sources are configured, ESP-IDF reports the first detected source. The firmware handles wake causes in this priority:

1. **GPIO/Button** - Highest (user interaction)
2. **LoRa Interrupt** - High (incoming message)
3. **PMU** - Medium (power event)
4. **Timer** - Low (scheduled wake)

---

## Platform-Specific ESP32 Code

### ESP32 Variant Detection

The code adapts to different ESP32 variants using ESP-IDF's SOC capability macros:

```cpp
#ifdef CONFIG_IDF_TARGET_ESP32
// Original ESP32
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2
// ESP32-S2
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S3
// ESP32-S3
#endif

#ifdef CONFIG_IDF_TARGET_ESP32C3
// ESP32-C3
#endif

#ifdef CONFIG_IDF_TARGET_ESP32C6
// ESP32-C6
#endif
```

### SOC Capability Macros

**1. RTC IO Hold Support** (`SOC_RTCIO_HOLD_SUPPORTED`)

Used in: `sleep.cpp:150`, `main-esp32.cpp:213`

```cpp
#if SOC_RTCIO_HOLD_SUPPORTED
gpio_hold_en((gpio_num_t)LORA_CS);
gpio_deep_sleep_hold_en();
// On wake:
gpio_deep_sleep_hold_dis();
gpio_hold_dis((gpio_num_t)LORA_CS);
#endif
```

**Purpose:** Maintain GPIO state during deep sleep to prevent current leakage

**Supported:** ESP32, ESP32-S2, ESP32-S3

**2. PM Support EXT Wakeup** (`SOC_PM_SUPPORT_EXT_WAKEUP`)

Used in: `main-esp32.cpp:233`, `sleep.cpp:514`

```cpp
#if SOC_PM_SUPPORT_EXT_WAKEUP
// Use newer gpio_wakeup API with pullup/pulldown
gpio_pulldown_en((gpio_num_t)LORA_DIO1);
gpio_wakeup_enable((gpio_num_t)LORA_DIO1, GPIO_INTR_HIGH_LEVEL);
#else
// Use older API
gpio_wakeup_enable((gpio_num_t)LORA_DIO1, GPIO_INTR_HIGH_LEVEL);
#endif
```

**Purpose:** Select appropriate GPIO wake API for platform

**Supported:** ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6

### RTC GPIO Isolation

**Purpose:** Prevent current leakage through unused RTC GPIOs during deep sleep

**Implementation:** `src/platform/esp32/main-esp32.cpp:214-227`

```cpp
#if SOC_RTCIO_HOLD_SUPPORTED
for (gpio_num_t gpio_num = GPIO_NUM_0; gpio_num < GPIO_NUM_MAX; gpio_num++) {
    // Check if GPIO is valid and RTC-capable
    if (GPIO_IS_VALID_GPIO(gpio_num) && rtc_gpio_is_valid_gpio(gpio_num)) {
        // Check if we're holding this pin (intentionally)
        bool is_held = gpio_hal_get_pin_hold(&GPIO_HAL, gpio_num);

        if (!is_held) {
            // Not held - isolate it to prevent leakage
            rtc_gpio_isolate(gpio_num);
        }
    }
}
#endif
```

**Effect:**
- Reduces deep sleep current significantly (10-100μA per pin)
- Essential for achieving < 1mA sleep current
- Only affects RTC domain GPIOs

### 32kHz External Crystal Support

**Purpose:** Use external 32.768kHz crystal for RTC instead of internal RC oscillator

**Benefits:**
- More accurate timekeeping during sleep
- Lower sleep current (~50-100μA savings)
- Better timer wake-up accuracy

**Implementation:** `src/platform/esp32/main-esp32.cpp:65-101`

```cpp
#ifdef HAS_32768HZ
void enableSlowCLK() {
    // Check if already using external crystal
    rtc_slow_freq_t current_freq = rtc_clk_slow_freq_get();
    if (current_freq == RTC_SLOW_FREQ_32K_XTAL) {
        LOG_INFO("32kHz XTAL already enabled\n");
        return;
    }

    // Select external 32kHz crystal
    rtc_clk_32k_enable(true);
    rtc_clk_slow_freq_set(RTC_SLOW_FREQ_32K_XTAL);

    // Wait for clock to stabilize
    vTaskDelay(pdMS_TO_TICKS(200));

    // Calibrate
    uint32_t cal_val = rtc_clk_cal(RTC_CAL_32K_XTAL, 1024);
    LOG_INFO("32kHz XTAL calibrated: %lu\n", cal_val);
}
#endif
```

**Hardware Requirements:**
- External 32.768kHz crystal on XTAL_32K_P (GPIO32) and XTAL_32K_N (GPIO33)
- Load capacitors (typically 10-20pF)
- Defined in variant.h with `HAS_32768HZ`

**Variants with 32kHz:**
- Most T-Beam variants
- Heltec V2, V3
- Many custom boards

### Modem Sleep / CPU Frequency Scaling

**Modem Sleep:** `src/sleep.cpp:481-503`

```cpp
#ifdef ARCH_ESP32
void enableModemSleep() {
    esp_pm_config_esp32_t config = {
        .max_freq_mhz = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ, // Usually 240
        .min_freq_mhz = 20,  // Minimum: 20MHz
        .light_sleep_enable = false // We control light sleep manually
    };

    esp_err_t err = esp_pm_configure(&config);
    if (err != ESP_OK) {
        LOG_WARN("Failed to configure power management: %d\n", err);
    }
}
#endif
```

**Effect:**
- Automatic CPU frequency scaling when idle
- Reduces power draw by ~20-30mA during idle
- No impact on performance (scales up when needed)

**CPU Frequency Control:** `src/sleep.cpp:69-96`

```cpp
void setCPUFast(bool on) {
    // WiFi needs 240MHz for stability
    if (isWifiAvailable()) {
        setCpuFrequencyMhz(240);
        return;
    }

    uint32_t freq = on ? 240 : 80;
    setCpuFrequencyMhz(freq);

    LOG_DEBUG("CPU frequency: %lu MHz\n", freq);
}
```

**Usage:**
- Boot/init: 240MHz
- Normal operation: 80MHz (sufficient for LoRa and BLE)
- WiFi enabled: 240MHz (required)

**Power Savings:**
- 240MHz → 80MHz: ~15-20mA reduction
- Combined with modem sleep: ~30-40mA total savings

---

## Configuration and Variants

### Device Role Impact on Sleep

**Router Role:**
```cpp
// PowerFSM.cpp:271, Default.h:41
if (config.device.role == meshtastic_Config_DeviceConfig_Role_ROUTER) {
    // Aggressive power-saving transitions
    default_wait_bluetooth_secs = 1      // Quick BT shutdown
    default_sds_secs = ONE_DAY           // 24h deep sleep
    default_ls_secs = ONE_DAY            // 24h light sleep
    default_screen_on_secs = 1           // Minimal screen time

    // LoRa wake enabled
    shouldLoraWake() = true;
}
```

**Tracker/Sensor Roles:**
```cpp
// PowerFSM.cpp:385-387
if (role == TRACKER || role == TAK_TRACKER || role == SENSOR) {
    // Disable automatic power-saving
    shouldEnterPowerSaving() = false;

    // Modules control their own sleep timing
    // Position updates, telemetry readings drive wake/sleep
}
```

**Client Role (default):**
```cpp
// Normal power-saving
default_wait_bluetooth_secs = 60        // Keep BT on for phone
default_sds_secs = UINT32_MAX           // No automatic deep sleep
default_ls_secs = 300                   // 5 min light sleep
default_screen_on_secs = 600            // 10 min screen on
```

**Repeater Role:**
```cpp
// Similar to Router
shouldLoraWake() = true;
// Always-on mesh participant
```

### Hardware Variant Configurations

Variants define hardware-specific sleep parameters in `variants/*/variant.h`:

**Example: Heltec Capsule Sensor V3**
```cpp
// variants/heltec_capsule_sensor_v3/variant.h:50-53
#define BUTTON_PIN 0
#define BUTTON_NEED_PULLUP
#define ESP32S3_WAKE_TYPE ESP_EXT1_WAKEUP_ANY_HIGH
```

**Example: T-Beam**
```cpp
// variants/tbeam/variant.h
#define BUTTON_PIN 38
#define HAS_32768HZ  // External 32kHz crystal
#define HAS_PMU      // AXP192/AXP2101
#define PMU_IRQ 35
```

**Example: Heltec WiFi LoRa 32 V3**
```cpp
// variants/heltec_v3/variant.h
#define BUTTON_PIN 0
#define VEXT_ENABLE_V03 36  // Display power control
// No PMU, GPIO-controlled peripherals
```

### Wake Type Overrides

Some ESP32-S3 variants require different wake logic due to hardware design:

**Standard (active-low button):**
```cpp
ESP_EXT1_WAKEUP_ANY_LOW  // Wake when any pin goes low
```

**Inverted (active-high button):**
```cpp
ESP_EXT1_WAKEUP_ANY_HIGH  // Wake when any pin goes high
```

**Variants with ESP32S3_WAKE_TYPE:**
- Heltec Capsule Sensor V3: `ESP_EXT1_WAKEUP_ANY_HIGH`
- Heltec Sensor Hub: `ESP_EXT1_WAKEUP_ANY_HIGH`

### Sleep Configuration Sources

**1. Compile-Time Defines (variant.h, configuration.h):**
```cpp
#define SLEEP_TIME 30          // Light sleep iteration (seconds)
#define BUTTON_PIN 38          // Wake button GPIO
#define BUTTON_NEED_PULLUP     // Enable internal pullup
#define HAS_32768HZ            // External 32kHz crystal
#define HAS_PMU                // Power management unit
```

**2. Default Configuration (Default.h):**
```cpp
default_wait_bluetooth_secs = IF_ROUTER(1, 60)
default_sds_secs = IF_ROUTER(ONE_DAY, UINT32_MAX)
default_ls_secs = IF_ROUTER(ONE_DAY, 300)
default_min_wake_secs = 10
default_screen_on_secs = IF_ROUTER(1, 600)
```

**3. User Configuration (config.power.*):**
```cpp
// Modifiable via AdminModule, persisted in NodeDB
config.power.ls_secs                    // Light sleep duration
config.power.sds_secs                   // Deep sleep duration
config.power.min_wake_secs              // Min wake time
config.power.wait_bluetooth_secs        // BT timeout
config.power.is_power_saving            // Enable power saving
config.power.on_battery_shutdown_after_secs  // Auto-shutdown
```

**Priority:** User config > Defaults > Compile-time defines

### Observable Pattern for Sleep Coordination

The firmware uses an observer pattern to coordinate sleep across subsystems.

**Preflight Sleep Observable** (`sleep.cpp:40`)

```cpp
Observable<const meshtastic_MeshPacket *> preflightSleep;

int32_t waitEnterSleep(bool deepsleep) {
    notifyObservers(&preflightSleep, (const meshtastic_MeshPacket *)NULL);

    // Wait up to 30 seconds for subsystems to veto sleep
    uint32_t start = millis();
    while ((preflightSleep.numObservers() > 0) &&
           (millis() - start < 30000)) {
        delay(100);
    }

    if (millis() - start >= 30000) {
        LOG_WARN("Sleep preflight timeout!\n");
        // FIXME: Bug #167 - should handle gracefully
        assert(0); // Currently restarts
    }
}
```

**Observers can veto sleep** by returning 1. Example:

```cpp
// RadioInterface.cpp:254
int32_t RadioInterface::notifyObserver(const meshtastic_MeshPacket *p) {
    if (isSending || isReceiving) {
        return 1; // Veto sleep - radio busy
    }
    return 0; // Allow sleep
}
```

**Deep Sleep Observable** (`sleep.cpp:43`)

```cpp
Observable<void *> notifyDeepSleep;

// Before deep sleep
notifyDeepSleep.notifyObservers(NULL);
```

**Observers prepare for shutdown:**
- RadioInterface: Powers down radio
- GPS: Saves state, powers down GPS
- Screen: Enters deep sleep mode
- AmbientLighting: Turns off LEDs

**Light Sleep Observables** (`sleep.cpp:50, 53`)

```cpp
Observable<void *> notifyLightSleep;
Observable<esp_sleep_wakeup_cause_t> notifyLightSleepEnd;

// Before light sleep
notifyLightSleep.notifyObservers(NULL);

// After wake
esp_sleep_wakeup_cause_t cause = doLightSleep(...);
notifyLightSleepEnd.notifyObserversWithArg(cause);
```

**Observers:**
- ButtonThread: Detach/reattach interrupts
- TFT displays: Enter/exit sleep mode
- Input drivers: Disable/enable interrupts

**Example: ButtonThread.cpp:111-112**
```cpp
notifyLightSleep.observe(&buttonThread,
    [](void *) {
        detachButtonInterrupts();
        return 0;
    });

notifyLightSleepEnd.observe(&buttonThread,
    [](esp_sleep_wakeup_cause_t cause) {
        attachButtonInterrupts();
        return 0;
    });
```

---

## Current Limitations and Gaps

### Known Issues

#### 1. Bug #167: Sleep Entry Timeout

**Location:** `src/sleep.cpp:187`

```cpp
if (millis() - start >= 30000) {
    LOG_WARN("Sleep preflight timeout - restarting\n");
    // FIXME #167: Better handling needed
    assert(0); // Causes restart
}
```

**Problem:** If a subsystem blocks sleep for >30 seconds, device restarts

**Root Cause:** Observer pattern timeout with no graceful degradation

**Impact:** Potential for unexpected restarts during legitimate radio activity

**Proposed Fix:**
- Cancel sleep attempt, return to normal operation
- Log which observer is blocking
- Retry sleep after backoff period

#### 2. Power State Verification Missing

**Location:** `src/PowerFSM.cpp:61`

```cpp
// FIXME: Make sure GPS and LORA radio are off first
// Goal: TBD mA sleep current
```

**Problem:** No verification that peripherals are actually powered down before deep sleep

**Impact:** Higher than expected sleep current

**Proposed Fix:**
- Check radio `isArmedForSleep()` before deep sleep
- Verify GPS power state
- Log power states before sleep

#### 3. Phone Packet Detection Race

**Location:** `src/PowerFSM.cpp:169`

```cpp
// FIXME: Should immediately send EVENT_PACKET_FOR_PHONE if queue not empty
```

**Problem:** Device may enter light sleep with packets waiting for phone

**Impact:** Delayed packet delivery, poor user experience

**Proposed Fix:**
- Check `toPhoneQueue` before entering light sleep
- Trigger `EVENT_PACKET_FOR_PHONE` if non-empty

#### 4. UART Wake Disabled

**Location:** `src/sleep.cpp:387-394`

```cpp
// UART wake causes issues on T-Beam when USB depowered
// UART wake limited to ~9600 bps with reference clock
// Commented out for now
/*
gpio_wakeup_enable((gpio_num_t)SERIAL_RX, GPIO_INTR_LOW_LEVEL);
esp_sleep_enable_uart_wakeup(SERIAL_NUM);
*/
```

**Problem:** Cannot wake from light sleep via serial activity

**Impact:** Must press button to wake for serial access

**Workaround:** Use button wake, then access serial

**Possible Fix:**
- Variant-specific UART wake enable
- Debounce spurious wake events
- Document baud rate limitations

#### 5. T-Watch S3 PMU Long Press IRQ

**Location:** `src/Power.cpp:853`

```cpp
// FIXME: Long press IRQ disabled - triggers incorrectly
pmu.disableIRQ(XPOWERS_AXP2101_PKEY_LONG_IRQ);
```

**Problem:** Long press detection unreliable on T-Watch S3

**Impact:** Cannot use long press for power off

**Status:** Hardware or PMU driver issue, needs investigation

#### 6. Light Sleep Default Parameter

**Location:** `src/sleep.cpp:353`

```cpp
// FIXME: Use a more reasonable default
esp_sleep_wakeup_cause_t doLightSleep(uint64_t sleepMsec)
```

**Problem:** Function signature lacks context for "reasonable default"

**Impact:** Callers must always specify sleep duration

**Proposed Fix:**
- Use `config.power.ls_secs` as default
- Document default behavior

#### 7. EXT1 Wake Status Not Read

**Location:** `src/sleep.cpp:129`

```cpp
case ESP_SLEEP_WAKEUP_EXT1:
    LOG_INFO("Wake from ext1 RTC_CNTL interrupt\n");
    // Could read: esp_sleep_get_ext1_wakeup_status()
    // Would tell us which GPIO caused wake
```

**Problem:** Cannot determine which GPIO caused wake in multi-GPIO scenarios

**Impact:** Debugging difficult, can't distinguish wake sources

**Proposed Fix:**
- Read and log `esp_sleep_get_ext1_wakeup_status()`
- Decode bitmask to GPIO numbers

### Missing Features

#### 1. No EXT0 Wake-up

**Status:** Supported by hardware, not implemented

**EXT0 Capabilities:**
- Single GPIO wake source
- Level-triggered (HIGH or LOW)
- Works on any RTC GPIO
- Lower latency than EXT1

**Potential Use Cases:**
- Simplified single-button wake
- Alternative if EXT1 pins unavailable
- Specific level detection (e.g., power good signal)

**Implementation Effort:** Low

**Example Code:**
```cpp
esp_sleep_enable_ext0_wakeup(GPIO_NUM_38, 0); // Wake on LOW
```

#### 2. No ULP Coprocessor Usage

**Status:** Wake cause supported (line 119), configuration not implemented

**ULP Capabilities:**
- Ultra-low power coprocessor (8-150μA)
- Can read sensors while main CPU sleeps
- Wake main CPU on threshold exceeded
- Program in assembly or C (newer ESP32 variants)

**Potential Use Cases:**
- Accelerometer monitoring (wake on movement)
- Temperature monitoring (wake on threshold)
- Battery monitoring (wake on voltage drop)
- Button debouncing
- Analog sensor reading

**Implementation Effort:** High (requires ULP programming)

**Power Savings:** Significant (can extend deep sleep indefinitely)

**Example:**
```cpp
// ULP monitors accelerometer, wakes main CPU on movement
ulp_set_wakeup_period(0, 100000); // 100ms sampling
esp_sleep_enable_ulp_wakeup();
```

#### 3. No Touchpad Wake

**Status:** Wake cause supported (line 116), not configured

**Touchpad Capabilities:**
- Some ESP32 GPIOs have capacitive touch sensing
- Wake on touch without mechanical button
- No external components needed

**Potential Use Cases:**
- Touch-to-wake on devices with touch-capable GPIOs
- Alternative to button in sealed enclosures
- Gesture detection (multiple touchpads)

**Implementation Effort:** Medium

**Hardware Requirements:**
- Touch-capable GPIO (ESP32: 0, 2, 4, 12-15, 27, 32-33)
- Exposed touch surface

**Example:**
```cpp
touch_pad_init();
touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
touchAttachInterrupt(T0, touchCallback, threshold);
esp_sleep_enable_touchpad_wakeup();
```

#### 4. No Wake Stub

**Status:** Not implemented

**Wake Stub Capabilities:**
- Code executes immediately on wake, before full boot
- Runs from RTC fast memory
- Can read sensors, update RTC memory
- Can return to deep sleep or continue boot

**Potential Use Cases:**
- Fast sensor sampling (millisecond wake time)
- Quick GPIO check before full boot
- Time-critical wake processing
- Battery check before booting

**Implementation Effort:** Medium-High

**Example:**
```cpp
RTC_DATA_ATTR int wake_count = 0;

void RTC_IRAM_ATTR wake_stub() {
    wake_count++;

    // Check battery
    if (battery_voltage() < CRITICAL_VOLTAGE) {
        esp_default_wake_deep_sleep(); // Continue boot
    } else {
        // Log and return to deep sleep
        esp_deep_sleep_start();
    }
}

void setup() {
    esp_set_deep_sleep_wake_stub(&wake_stub);
}
```

#### 5. No WiFi Beacon Wake

**Status:** Not implemented (ESP-IDF supports it)

**WiFi Beacon Wake Capabilities:**
- Maintain WiFi connection during light sleep
- Wake on beacon interval to receive data
- Average power: ~15-30mA (vs ~80mA always on)

**Limitations:**
- Only works with light sleep
- Requires stable AP connection
- Not compatible with mesh timing

**Potential Use Cases:**
- MQTT-connected devices
- WiFi-based trackers
- Home automation integration

**Implementation Effort:** Medium

**Compatibility:** Would require careful integration with mesh timing

---

## Future Development Opportunities

### High-Impact Enhancements

#### 1. ULP Accelerometer Monitoring

**Goal:** Wake from deep sleep only on device movement

**Benefits:**
- 99% power reduction for stationary devices
- Days/weeks battery life for mounted nodes
- Mesh-aware: wake to check for messages periodically

**Implementation Plan:**
1. ULP program reads accelerometer via I2C every 100ms
2. Compare reading to threshold
3. Wake main CPU if movement detected
4. Main CPU processes movement, sends position update

**Power Profile:**
- Stationary: 10-50μA (ULP only)
- Moving: Normal operation (~60-100mA)

**Compatible Sensors:**
- LIS3DH (I2C, very low power)
- MPU6050 (I2C)
- ADXL345 (I2C)

**Code Sketch:**
```cpp
// ULP assembly
.global entry
entry:
    // Read accelerometer X register via I2C
    I2C_RD 0x28, 0x18, accel_x

    // Compare to threshold
    MOVE R0, accel_x
    JUMP wake_cpu, gt, threshold

    // No movement, return to sleep
    HALT

wake_cpu:
    WAKE
    HALT
```

#### 2. Intelligent Sleep Scheduling

**Goal:** Coordinate sleep with mesh activity and sensor timing

**Current Behavior:**
- Fixed sleep intervals
- May wake when no mesh activity expected
- Sensor modules independently trigger wake

**Enhanced Behavior:**
- Calculate next expected mesh packet time
- Align wake with position/telemetry update needs
- Skip unnecessary wake cycles

**Implementation:**
```cpp
uint32_t calculateOptimalSleepTime() {
    uint32_t nextMeshPacket = getMeshInterval() + getJitter();
    uint32_t nextPositionUpdate = positionModule->getNextUpdateTime();
    uint32_t nextTelemetry = telemetryModule->getNextUpdateTime();

    uint32_t nextWake = min(nextMeshPacket,
                        min(nextPositionUpdate, nextTelemetry));

    // Don't wake just for mesh if no other reason
    if (nextWake == nextMeshPacket &&
        role != ROUTER && role != REPEATER) {
        nextWake = min(nextPositionUpdate, nextTelemetry);
    }

    return nextWake;
}
```

**Power Savings:** 10-30% reduction in wake cycles

#### 3. Wake Stub for Fast Battery Check

**Goal:** Check battery voltage on wake without full boot

**Benefits:**
- Prevent boot attempts with dead battery
- Log battery trends without main CPU
- Fast critical voltage shutdown

**Implementation:**
```cpp
RTC_DATA_ATTR uint16_t battery_samples[100];
RTC_DATA_ATTR uint8_t sample_index = 0;

void RTC_IRAM_ATTR wake_stub() {
    // Read battery ADC
    uint16_t battery_raw = READ_PERI_REG(SENS_SAR_MEAS_START1_REG);
    battery_samples[sample_index++] = battery_raw;

    if (sample_index >= 100) sample_index = 0;

    // Check critical voltage
    if (battery_raw < CRITICAL_RAW_VALUE) {
        // Too low, return to deep sleep indefinitely
        esp_deep_sleep_start();
        // Does not return
    }

    // OK to boot
    esp_default_wake_deep_sleep();
}
```

**Power Savings:** Avoid costly boot cycles when battery dead

#### 4. Multi-Button Wake Support

**Goal:** Wake on any of multiple buttons (up/down/enter/back)

**Current:** Only single button wake (BUTTON_PIN)

**Enhanced:**
```cpp
uint64_t gpioMask = 0;
if (BUTTON_PIN) gpioMask |= (1ULL << BUTTON_PIN);
if (ENCODER_BTN) gpioMask |= (1ULL << ENCODER_BTN);
if (BACK_BUTTON) gpioMask |= (1ULL << BACK_BUTTON);

esp_sleep_enable_ext1_wakeup_io(gpioMask, ESP_EXT1_WAKEUP_ANY_LOW);

// On wake:
uint64_t wakeup_pin = esp_sleep_get_ext1_wakeup_status();
// Decode which button was pressed
```

**Use Cases:**
- Navigate menu from deep sleep
- Multi-function wake (long press vs short press)
- Accessibility (multiple wake methods)

#### 5. Capacitive Touch Wake

**Goal:** Touch-to-wake on devices with exposed touch pads

**Benefits:**
- No mechanical button wear
- Sealed enclosure designs
- Multi-touch gestures

**Implementation:**
```cpp
void setupTouchWake() {
    touch_pad_init();
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5,
                          TOUCH_HVOLT_ATTEN_1V);

    // Configure touch pad (e.g., GPIO 4 = T0)
    touch_pad_config(TOUCH_PAD_NUM0, TOUCH_THRESHOLD);

    esp_sleep_enable_touchpad_wakeup();
}
```

**Hardware Requirements:**
- Touch-capable GPIOs on variant
- Exposed conductive surface
- Touch pad tuning per hardware

### Cross-Platform Improvements

#### 1. Unified Sleep API

**Problem:** Platform-specific sleep implementations

**Solution:** Abstract sleep interface

```cpp
class PlatformSleep {
public:
    virtual void enterDeepSleep(uint64_t msec) = 0;
    virtual void enterLightSleep(uint64_t msec) = 0;
    virtual void configureGPIOWake(uint8_t pin, bool level) = 0;
    virtual void configureTimerWake(uint64_t msec) = 0;
    virtual WakeCause getWakeCause() = 0;
};

class ESP32Sleep : public PlatformSleep { ... };
class NRF52Sleep : public PlatformSleep { ... };
class RP2040Sleep : public PlatformSleep { ... };
```

**Benefits:**
- Consistent behavior across platforms
- Easier to add new platforms
- Better testing

#### 2. NRF52 Deep Sleep Fix

**Current:** Deep sleep disabled on NRF52 (Power.cpp:802-807)

**Problem:** "Freezing the board"

**Investigation Needed:**
- Test on multiple nRF52 variants
- Check softdevice compatibility
- Verify GPIO configuration

**Potential Fix:**
- Properly shutdown softdevice before sleep
- Save/restore radio state
- Test with sd_power_system_off()

#### 3. RP2040 Sleep Implementation

**Current:** Limited sleep support on RP2040

**RP2040 Capabilities:**
- Sleep mode (CPU halted, peripherals running)
- Dormant mode (most clocks stopped, < 1mA)
- Wake on GPIO interrupt

**Implementation Needed:**
- Port light sleep to RP2040 sleep mode
- Port deep sleep to dormant mode
- Test power consumption

### Power Optimization Strategies

#### 1. Peripheral Power Profiling

**Goal:** Measure actual sleep current for each variant

**Method:**
1. Hardware: μCurrent or similar current meter
2. Test: Measure sleep current with/without each peripheral
3. Document: Create power budget table

**Deliverable:**
```markdown
| Variant | Deep Sleep | Light Sleep | Active |
|---------|------------|-------------|---------|
| T-Beam  | 0.8mA      | 2.1mA       | 95mA    |
| Heltec V3 | 0.6mA    | 1.8mA       | 85mA    |
...
```

**Use:** Inform design decisions, debug high sleep current

#### 2. Dynamic Peripheral Shutdown

**Goal:** Power down unused peripherals based on role and config

**Examples:**
```cpp
// No GPS needed for router
if (role == ROUTER) {
    disableGPS();
    power_down_gps_rail();
}

// No screen needed for sensor
if (role == SENSOR && !config.display.enabled) {
    disableScreen();
    power_down_display_rail();
}

// No Bluetooth if WiFi MQTT only
if (mqtt_enabled && !phone_paired) {
    disableBluetooth();
}
```

**Power Savings:** 5-50mA per peripheral

#### 3. Adaptive Sleep Timing

**Goal:** Learn optimal sleep intervals based on mesh activity

**Method:**
- Track packet arrival times
- Calculate statistical intervals
- Adjust sleep timing dynamically

**Example:**
```cpp
class MeshActivityLearner {
    uint32_t packet_times[100];
    uint8_t index = 0;

    void recordPacket() {
        packet_times[index++] = millis();
        if (index >= 100) index = 0;
    }

    uint32_t getAverageInterval() {
        // Calculate average time between packets
        // Return optimal sleep interval
    }
};
```

**Power Savings:** Reduce unnecessary wake cycles by 10-30%

---

## Developer Guide

### Adding a New Wake Source

**Example: Adding RTC_IO (EXT0) Wake-up**

**1. Add Configuration**

```cpp
// variant.h
#define WAKEUP_GPIO GPIO_NUM_34  // RTC-capable GPIO
#define WAKEUP_LEVEL 0            // Wake on LOW
```

**2. Configure Wake Source**

```cpp
// src/sleep.cpp or src/platform/esp32/main-esp32.cpp

void configureExt0Wake() {
    #ifdef WAKEUP_GPIO
    gpio_num_t gpio = (gpio_num_t)WAKEUP_GPIO;
    int level = WAKEUP_LEVEL;

    // EXT0 supports internal pullup/pulldown
    if (level == 0) {
        gpio_pullup_en(gpio);
    } else {
        gpio_pulldown_en(gpio);
    }

    // Enable EXT0 wake
    esp_sleep_enable_ext0_wakeup(gpio, level);

    LOG_INFO("EXT0 wake configured: GPIO %d, level %d\n", gpio, level);
    #endif
}

// Call before deep sleep
void cpuDeepSleep(uint64_t msecToWake) {
    // ... existing code ...

    configureExt0Wake();

    esp_deep_sleep_start();
}
```

**3. Handle Wake Cause**

```cpp
// src/sleep.cpp - initDeepSleep()

case ESP_SLEEP_WAKEUP_EXT0:
    LOG_INFO("Wake from EXT0 on GPIO %d\n", WAKEUP_GPIO);

    // Optional: Read pin state to verify
    int pin_state = gpio_get_level((gpio_num_t)WAKEUP_GPIO);
    LOG_DEBUG("Pin state on wake: %d\n", pin_state);

    // Trigger event or set flag
    wakeFromExternalSignal = true;
    break;
```

**4. Document**

Update variant README and CLAUDE.md with new wake source.

### Testing Sleep/Wake Functionality

**Serial Monitor Testing:**

```cpp
// Add debug prints
LOG_INFO("Entering deep sleep for %lu seconds\n", secs);
LOG_INFO("Wake sources: Timer=%d, EXT1=0x%llx\n",
         timer_enabled, ext1_mask);

// On wake:
LOG_INFO("Woke from: %s\n", wakeReasonToString(wakeCause));
```

**Power Measurement Testing:**

1. **Hardware Setup:**
   - μCurrent or INA219 in series with battery
   - Log current readings via serial or I2C

2. **Test Sequence:**
   ```cpp
   // Measure each state
   enterState(ON);     delay(10000);  // 10s sample
   enterState(DARK);   delay(10000);
   enterState(LS);     delay(60000);  // 60s sample
   enterState(SDS);    delay(300000); // 5min sample
   ```

3. **Verify:**
   - Deep sleep: < 1mA
   - Light sleep: 1-5mA
   - DARK: < 80mA
   - ON: < 120mA

**Wake Source Testing:**

```cpp
void testWakeSources() {
    LOG_INFO("=== Wake Source Test ===\n");

    // Test 1: Button wake
    LOG_INFO("Test 1: Press button to wake from deep sleep\n");
    doDeepSleep(60000); // 60s timeout
    // Should wake on button press before timeout
    LOG_INFO("Woke from: %d\n", wakeCause);
    assert(wakeCause == ESP_SLEEP_WAKEUP_EXT1);

    // Test 2: Timer wake
    LOG_INFO("Test 2: Timer wake (don't press button)\n");
    doDeepSleep(10000); // 10s timeout
    LOG_INFO("Woke from: %d\n", wakeCause);
    assert(wakeCause == ESP_SLEEP_WAKEUP_TIMER);

    // Test 3: LoRa wake (need second device to send)
    LOG_INFO("Test 3: LoRa wake (send packet from another device)\n");
    enableLoraInterrupt();
    doLightSleep(60000);
    LOG_INFO("Woke from: %d\n", wakeCause);
    // Should be ESP_SLEEP_WAKEUP_GPIO if packet received

    LOG_INFO("=== Tests Complete ===\n");
}
```

### Common Pitfalls

#### 1. Non-RTC GPIO for Deep Sleep

**Problem:** Using non-RTC GPIO for EXT1 wake

**Symptom:** Wake doesn't work, device sleeps indefinitely

**Solution:** Check RTC GPIO list for your variant
- ESP32: 0, 2, 4, 12-15, 25-27, 32-39
- ESP32-S2: 0-21
- ESP32-S3: 0-21
- ESP32-C3: 0-5

**Verification:**
```cpp
bool is_rtc = rtc_gpio_is_valid_gpio((gpio_num_t)MY_GPIO);
if (!is_rtc) {
    LOG_ERROR("GPIO %d is not RTC-capable!\n", MY_GPIO);
}
```

#### 2. Forgetting GPIO Hold

**Problem:** Not holding critical GPIOs during deep sleep

**Symptom:** High sleep current, peripherals powered

**Solution:** Hold GPIOs that control power or must maintain state

```cpp
// Before deep sleep
gpio_hold_en((gpio_num_t)LORA_CS);      // Keep CS high
gpio_hold_en((gpio_num_t)POWER_EN);     // Keep power on/off
gpio_deep_sleep_hold_en();

// After wake
gpio_deep_sleep_hold_dis();
gpio_hold_dis((gpio_num_t)LORA_CS);
gpio_hold_dis((gpio_num_t)POWER_EN);
```

#### 3. Not Disabling Peripherals

**Problem:** Leaving I2C, SPI, UART active before sleep

**Symptom:** High sleep current (10-50mA extra)

**Solution:** Explicitly disable peripherals

```cpp
// Before sleep
Wire.end();           // I2C
SPI.end();            // SPI
Serial.end();         // UART (optional, may need serial debug)
esp_wifi_stop();      // WiFi
esp_bluedroid_disable(); // Bluetooth

// Peripheral power control
digitalWrite(GPS_POWER_EN, LOW);
digitalWrite(LORA_POWER_EN, LOW);
```

#### 4. Race Condition on Wake

**Problem:** Handling wake cause before peripherals reinitialize

**Symptom:** Crashes, hangs on wake

**Solution:** Proper initialization order

```cpp
void onWake() {
    // 1. Determine wake cause
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    // 2. Reinitialize critical peripherals
    Wire.begin();
    SPI.begin();

    // 3. Release GPIO holds
    gpio_deep_sleep_hold_dis();

    // 4. Reinitialize subsystems
    initRadio();
    initGPS();

    // 5. Handle wake cause
    switch (cause) {
        case ESP_SLEEP_WAKEUP_EXT1:
            handleButtonWake();
            break;
        // ...
    }
}
```

#### 5. Button Bounce on Wake

**Problem:** Button bounce causes multiple wake events

**Symptom:** Enters light sleep, immediately wakes again

**Solution:** Debounce on wake

```cpp
// After wake from GPIO
if (cause == ESP_SLEEP_WAKEUP_GPIO) {
    delay(50); // Debounce delay

    int button_state = digitalRead(BUTTON_PIN);
    if (button_state == BUTTON_ACTIVE_STATE) {
        // Still pressed - valid wake
        handleButtonPress();
    } else {
        // Released already - probably bounce, ignore
        LOG_DEBUG("Spurious GPIO wake (bounce)\n");
        return; // May return to sleep
    }
}
```

### Debugging Sleep Issues

**Enable Verbose Logging:**

```cpp
// platformio.ini
build_flags =
    ${env.build_flags}
    -DLOG_LEVEL=LOG_LEVEL_DEBUG
    -DSLEEP_DEBUG=1
```

**Common Issues and Solutions:**

| Symptom | Likely Cause | Check |
|---------|--------------|-------|
| High sleep current (>10mA) | Peripheral not disabled | Measure current, disable one at a time |
| Won't wake from button | Non-RTC GPIO or wrong level | Verify GPIO is RTC-capable, check button wiring |
| Won't wake from timer | Timer not configured | Check `esp_sleep_enable_timer_wakeup()` called |
| Immediate wake | Button bounce or IRQ still asserted | Add debounce, check IRQ pin state |
| Crash on wake | GPIO holds not released | Release holds before accessing peripherals |
| Deep sleep timeout | Observer blocking sleep | Check preflight timeout, log blocking observer |

**Power Debugging Script:**

```python
# scripts/measure_sleep_current.py
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)

states = {
    'ON': [],
    'DARK': [],
    'LS': [],
    'SDS': []
}

current_state = 'ON'

while True:
    line = ser.readline().decode('utf-8')

    # Parse power measurements from device
    if 'Current:' in line:
        ma = float(line.split(':')[1].strip().replace('mA', ''))
        states[current_state].append(ma)

    # Detect state changes
    if 'State transition:' in line:
        current_state = line.split(':')[1].strip()

    # Print averages periodically
    if len(states[current_state]) >= 100:
        avg = sum(states[current_state]) / len(states[current_state])
        print(f"{current_state}: {avg:.2f} mA average")
        states[current_state] = []
```

---

## References

### Key Files and Line Numbers

**Sleep Implementation:**
- `src/sleep.cpp` - Main sleep logic (546 lines)
  - `doDeepSleep()`: line 198-345
  - `doLightSleep()`: line 353-470
  - `initDeepSleep()`: line 99-166
  - `enableLoraInterrupt()`: line 511-544
- `src/sleep.h` - Sleep API (55 lines)
- `src/platform/esp32/main-esp32.cpp` - ESP32 deep sleep (266 lines)
  - `cpuDeepSleep()`: line 197-265
  - GPIO isolation: line 214-227
  - EXT1 config: line 231-258

**Power Management:**
- `src/PowerFSM.cpp` - State machine (632 lines)
  - State definitions: line 58-255
  - Transition logic: line 270-338
  - Power-saving check: line 384-405
- `src/PowerFSM.h` - FSM interface (81 lines)
  - Events: line 18-42
- `src/Power.cpp` - Battery and PMU (1058 lines)
  - Low battery handling: line 802-807
  - PMU configuration: line 931-1057

**Configuration:**
- `src/mesh/Default.h` - Default values (lines 131-160)
- `variants/*/variant.h` - Hardware-specific configs

**Button/Input:**
- `src/ButtonThread.cpp` - Button interrupt handling (457 lines)
  - Interrupt attach/detach: line 337-371
  - Sleep observers: line 400-418

### ESP-IDF Documentation

**Sleep Modes:**
- [ESP32 Sleep Modes](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html)
- [Power Management](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/power_management.html)

**Wake Sources:**
- [EXT0/EXT1 Wake](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html#ext0-wakeup)
- [GPIO Wake](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#gpio-wakeup)
- [Timer Wake](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html#timer-wakeup)
- [ULP Wake](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ulp.html)

**Low-Level APIs:**
- [RTC IO](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html#rtc-io)
- [PM Configuration](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/power_management.html#api-reference)

### Related Issues and Pull Requests

**Known Issues:**
- [Bug #167](https://github.com/meshtastic/firmware/issues/167) - Sleep preflight timeout handling
- NRF52 deep sleep freezing (Power.cpp:802 comment)

**Relevant PRs:**
- Search GitHub for PRs with labels: `power`, `sleep`, `ESP32`

### External Resources

**Power Measurement Tools:**
- [μCurrent Gold](https://www.eevblog.com/product/ucurrent/) - Precision current measurement
- [Nordic Power Profiler Kit II](https://www.nordicsemi.com/Products/Development-hardware/Power-Profiler-Kit-2) - Real-time power profiling
- INA219/INA226 - I2C current sensors for embedded measurement

**ESP32 Sleep Optimization Guides:**
- [ESP32 Deep Sleep with External Wake-up](https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/)
- [ESP32 Low Power Design](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/low-power-mode.html)

**Meshtastic Resources:**
- [Meshtastic Documentation](https://meshtastic.org/docs/)
- [Firmware Development Guide](https://meshtastic.org/docs/development/firmware/)
- [Discord #firmware-dev](https://discord.com/invite/ktMAKGBnBs)

---

## Document Changelog

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-10-30 | Initial research compilation |
| 1.1 | 2025-10-30 | Added TreeGuard module deep sleep issue documentation |

---

## Branch-Specific Issues

### treeguard-module Branch: Deep Sleep Implementation Bug

**Status:** ⚠️ **BROKEN - Requires immediate fix**

**Location:** `src/modules/treeguard/TreeGuardModule.cpp`

#### Problem Description

The TreeGuard module (a vibration detection module for tree monitoring) has a **fundamentally broken deep sleep implementation** that bypasses all Meshtastic power management infrastructure. The module directly calls ESP-IDF sleep functions without proper coordination.

#### Current Implementation (Lines 146-166)

```cpp
void TreeGuardModule::goToDeepSleep()
{
    Wire.end();
    TG_vextOff();
    delay(30);

    if (rtc_gpio_is_valid_gpio((gpio_num_t)TG_INT_PIN)) {
        pinMode(TG_INT_PIN, INPUT_PULLDOWN);
        rtc_gpio_pullup_dis((gpio_num_t)TG_INT_PIN);
        rtc_gpio_pulldown_en((gpio_num_t)TG_INT_PIN);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)TG_INT_PIN, 1);
    } else {
        Serial.println("[TreeGuard] EXT0 disabled: INT pin not RTC-capable");
    }

    esp_sleep_enable_timer_wakeup((uint64_t)STATUS_UPDATE_INTERVAL_S * uS_TO_S_FACTOR);
    Serial.println("[TreeGuard] Deep sleep start");
    Serial.flush();
    delay(100);
    esp_deep_sleep_start(); // ❌ WRONG: Bypasses Meshtastic infrastructure
}
```

**Called from runOnce() after:**
```cpp
delay(4000); // ❌ WRONG: Crude wait for radio TX
goToDeepSleep();
```

#### Specific Issues

1. **Bypasses PowerFSM**
   - Calls `esp_deep_sleep_start()` directly (line 165)
   - Should use `doDeepSleep()` from `src/sleep.cpp`
   - PowerFSM state machine never transitions to SDS state
   - Other subsystems unaware device is entering deep sleep

2. **No Observable Pattern Coordination**
   - Doesn't call `waitEnterSleep()` - radio may be transmitting
   - Doesn't notify `notifyDeepSleep` observers
   - Radio, GPS, screen, and other subsystems not prepared for shutdown
   - **Radio transmission likely interrupted mid-packet**

3. **No NodeDB Persistence**
   - Doesn't call `nodeDB->saveToDisk()` before sleep
   - Risk of data loss on every sleep cycle
   - Configuration changes may be lost

4. **Incomplete Peripheral Shutdown**
   - Only calls `Wire.end()` and `TG_vextOff()`
   - Doesn't shut down:
     - GPS (if present)
     - Screen/display
     - Bluetooth
     - WiFi (if enabled)
     - Other I2C/SPI peripherals
     - PMU (if present)

5. **Radio Timing Issue**
   - Uses `delay(4000)` hoping radio transmission completes
   - No actual verification radio is done
   - May sleep during transmission, corrupting packet
   - Should use `preflightSleep` observable to wait for radio

6. **Missing GPIO Hold Configuration**
   - Doesn't hold critical GPIOs during deep sleep
   - May have high sleep current due to floating pins
   - LoRa CS pin should be held high

7. **No RTC GPIO Isolation**
   - Doesn't isolate unused RTC GPIOs
   - Will have higher sleep current (10-100μA per pin)
   - See `cpuDeepSleep()` implementation in `main-esp32.cpp:214-227`

#### Impact

- **Radio packets corrupted or dropped** - sleep before TX completes
- **Data loss** - NodeDB not saved before sleep
- **High sleep current** - peripherals not properly shut down
- **System instability** - other subsystems not prepared for sleep
- **Race conditions** - no coordination with mesh activity
- **Potential GPS data loss** - GPS not properly shut down

#### Required Fix

The TreeGuard module must be refactored to integrate with the existing Meshtastic sleep infrastructure:

**Option 1: Use doDeepSleep() (Recommended)**

```cpp
#include "sleep.h"
#include "PowerFSM.h"

void TreeGuardModule::goToDeepSleep()
{
    // 1. Signal completion to system
    sendText("SLEEP"); // Optional: notify going to sleep

    // 2. Wait for packet to be sent (observable pattern handles this)
    // Let the system detect we're done and enter sleep naturally

    // 3. Transition PowerFSM to deep sleep
    powerFSM.trigger(EVENT_SHUTDOWN);

    // OR: Call Meshtastic's deep sleep directly
    // doDeepSleep(STATUS_UPDATE_INTERVAL_S * 1000ULL);

    // Note: Should configure EXT0 wake before this:
    if (rtc_gpio_is_valid_gpio((gpio_num_t)TG_INT_PIN)) {
        esp_sleep_enable_ext0_wakeup((gpio_num_t)TG_INT_PIN, 1);
    }
}

int32_t TreeGuardModule::runOnce()
{
    // ... existing code ...

    if (wake == ESP_SLEEP_WAKEUP_EXT0) {
        processVibrationWake();
    } else {
        processTimerWake();
    }

    // Remove the crude delay
    // Let the observable pattern handle radio completion
    goToDeepSleep();

    return 0;
}
```

**Option 2: Register as Sleep Observer (Better)**

```cpp
// In constructor
TreeGuardModule::TreeGuardModule()
    : SinglePortModule("treeguard", meshtastic_PortNum_TEXT_MESSAGE_APP)
{
    // Register to be notified before sleep
    preflightSleep.observe(this, &TreeGuardModule::onPreflightSleep);
}

int32_t TreeGuardModule::onPreflightSleep(const meshtastic_MeshPacket *p)
{
    if (isSendingPacket) {
        return 1; // Veto sleep - packet not sent yet
    }
    return 0; // Allow sleep
}

void TreeGuardModule::onPacketSent()
{
    isSendingPacket = false;
    // Now safe to sleep
}
```

**Option 3: Use Power Module API**

```cpp
#include "Power.h"

void TreeGuardModule::requestSleep()
{
    // Configure accelerometer wake
    if (rtc_gpio_is_valid_gpio((gpio_num_t)TG_INT_PIN)) {
        esp_sleep_enable_ext0_wakeup((gpio_num_t)TG_INT_PIN, 1);
    }

    // Use Meshtastic's sleep function
    // This handles all coordination, NodeDB save, peripheral shutdown
    config.power.sds_secs = STATUS_UPDATE_INTERVAL_S;
    powerFSM.trigger(EVENT_SHUTDOWN);
}
```

#### Testing Required After Fix

1. **Verify packet transmission:**
   - Use second device to confirm packets arrive
   - Check packet integrity
   - Verify no transmission interruptions

2. **Verify wake functionality:**
   - Test EXT0 wake (accelerometer interrupt)
   - Test timer wake (24h status updates)
   - Verify wake cause detection

3. **Verify NodeDB persistence:**
   - Change configuration before sleep
   - Verify changes persist after wake

4. **Measure sleep current:**
   - Should be < 1mA in deep sleep
   - Test with/without peripheral shutdown
   - Verify GPIO holds working correctly

5. **Test system stability:**
   - Multiple sleep/wake cycles
   - Different wake sources
   - Check for memory leaks or crashes

#### References

- **Correct deep sleep implementation:** `src/sleep.cpp:198-345` (`doDeepSleep()`)
- **PowerFSM integration:** `src/PowerFSM.cpp`
- **Observable pattern:** `src/sleep.cpp:40-53`
- **ESP32 sleep example:** `src/platform/esp32/main-esp32.cpp:197-265` (`cpuDeepSleep()`)

#### Priority

**HIGH** - This should be fixed before the TreeGuard module is merged to master or deployed to production. Current implementation risks data loss, packet corruption, and unstable operation.

---

## Appendix: Quick Reference

### Sleep API Quick Reference

```cpp
// Deep sleep (ESP32)
doDeepSleep(60000);  // Sleep 60 seconds

// Light sleep (ESP32)
esp_sleep_wakeup_cause_t cause = doLightSleep(30000);  // Sleep 30 seconds

// Configure wake sources
esp_sleep_enable_timer_wakeup(60000000ULL);  // μs
esp_sleep_enable_ext1_wakeup(gpioMask, ESP_EXT1_WAKEUP_ANY_LOW);
gpio_wakeup_enable((gpio_num_t)pin, GPIO_INTR_LOW_LEVEL);
esp_sleep_enable_gpio_wakeup();

// Get wake cause
esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

// Power FSM
powerFSM.trigger(EVENT_PRESS);         // Trigger event
State current = powerFSM.getState();    // Get current state
```

### Configuration Quick Reference

```cpp
// User configuration (via AdminModule or config file)
config.power.is_power_saving = true;
config.power.ls_secs = 300;               // 5 minutes
config.power.sds_secs = UINT32_MAX;       // Disabled
config.power.min_wake_secs = 10;
config.power.wait_bluetooth_secs = 60;
config.display.screen_on_secs = 600;      // 10 minutes

// Device role (affects power behavior)
config.device.role = meshtastic_Config_DeviceConfig_Role_ROUTER;
```

### Power State Reference

```cpp
// Typical current draw (ESP32)
ON:         80-120 mA
DARK:       60-80 mA
NB:         40-60 mA
LS:         1-5 mA
SDS:        < 1 mA

// State transitions
ON → (timeout) → DARK → (timeout) → LS → (wake) → repeat
ANY → (low battery) → LowBattSDS
```

### RTC GPIO Reference

**ESP32:**
- RTC GPIOs: 0, 2, 4, 12, 13, 14, 15, 25, 26, 27, 32-39
- Touch GPIOs: 0, 2, 4, 12, 13, 14, 15, 27, 32, 33

**ESP32-S2:**
- RTC GPIOs: 0-21

**ESP32-S3:**
- RTC GPIOs: 0-21

**ESP32-C3:**
- RTC GPIOs: 0-5

---

**End of Document**
