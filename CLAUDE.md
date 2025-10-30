# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Meshtastic is an open-source LoRa mesh networking firmware for long-range, low-power communication without internet or cellular infrastructure. The firmware supports multiple hardware platforms: ESP32, nRF52, RP2040/RP2350, STM32WL, and Linux-based devices (Portduino).

## Build System

This project uses PlatformIO for building firmware across multiple hardware platforms.

### Building Firmware

**Important**: Always source the profile before running PlatformIO commands:
```bash
source ~/.profile && platformio <command>
```

**Build for specific target:**
```bash
source ~/.profile && platformio run -e <environment>
```

Common environments:
- `tbeam` (default) - ESP32 T-Beam
- `heltec_v3` - Heltec WiFi LoRa 32 V3
- `rak4631` - RAK WisBlock Core nRF52840
- `rpipico` - Raspberry Pi Pico RP2040

**List all available environments:**
```bash
source ~/.profile && platformio project config
```

**Architecture-specific build scripts:**
- `bin/build-esp32.sh <environment>` - Build ESP32 variants
- `bin/build-nrf52.sh <environment>` - Build nRF52 variants
- `bin/build-rpi2040.sh <environment>` - Build RP2040 variants
- `bin/build-stm32.sh <environment>` - Build STM32WL variants

**Build filesystem (ESP32 only):**
```bash
source ~/.profile && platformio run -e <environment> -t buildfs
```

**Upload firmware:**
```bash
source ~/.profile && platformio run -e <environment> -t upload
```

**Clean build:**
```bash
source ~/.profile && platformio run -e <environment> -t clean
```

### Testing

**Run tests:**
```bash
source ~/.profile && platformio test -e <environment>
```

Test files are located in `test/` directory.

### Code Quality

**Format code using Trunk:**
```bash
trunk fmt
```

The project uses Trunk for code formatting (configured in `.vscode/settings.json`). Format on save is enabled by default in VS Code.

**Static analysis:**
```bash
source ~/.profile && platformio check -e <environment>
```

Uses cppcheck with suppressions defined in `suppressions.txt`.

## Architecture

### Source Structure

- **`src/`** - Main firmware source code
  - **`main.cpp`** - Entry point and hardware initialization
  - **`mesh/`** - Core mesh networking implementation
    - `MeshService.cpp/h` - Top-level mesh service orchestration
    - `Router.cpp/h` - Message routing logic (FloodingRouter, ReliableRouter, NextHopRouter)
    - `RadioInterface.cpp/h` - Hardware abstraction for LoRa radios
    - `NodeDB.cpp/h` - Database of mesh nodes and their state
    - `Channels.cpp/h` - Channel configuration and encryption
    - `generated/` - Protocol buffer definitions (auto-generated)
    - `api/` - API interfaces (WiFi, Ethernet, Phone)
    - `http/` - Web server implementation
    - `wifi/` - WiFi client/AP functionality
    - `eth/` - Ethernet functionality
  - **`modules/`** - Plugin modules extending base functionality
    - Each module inherits from `MeshModule` or `ProtobufModule`
    - Examples: `AdminModule`, `TelemetryModule`, `PositionModule`, `TextMessageModule`
  - **`platform/`** - Platform-specific code
    - `esp32/` - ESP32 platform implementation
    - `nrf52/` - nRF52 platform implementation
    - `rp2xx0/` - RP2040/RP2350 implementation
    - `stm32wl/` - STM32WL implementation
    - `portduino/` - Linux/simulator implementation
  - **`gps/`** - GPS functionality
  - **`graphics/`** - Display drivers and UI
  - **`concurrency/`** - Threading and synchronization primitives
  - **`power/`** - Power management and sleep modes
- **`variants/`** - Hardware variant definitions (125+ variants)
  - Each variant has its own directory with:
    - `platformio.ini` - Build configuration
    - `variant.h` - Pin mappings and hardware features
- **`arch/`** - Architecture-specific PlatformIO configurations
  - `esp32/` - ESP32 architecture settings
  - `nrf52/` - nRF52 architecture settings
  - `rp2xx0/` - RP2040 architecture settings
  - `stm32/` - STM32 architecture settings
- **`bin/`** - Build scripts and utilities
- **`protobufs/`** - Protocol buffer definitions (currently empty, regenerated from upstream)

### Key Architectural Concepts

**Radio Interfaces**: All LoRa radio hardware is abstracted through `RadioInterface` base class. Specific implementations:
- `SX1262Interface`, `SX1268Interface`, `SX1280Interface` - Semtech chips
- `RF95Interface` - RFM95/96/97/98 modules
- `LLCC68Interface` - LLCC68 chip
- `LR1110Interface`, `LR1120Interface`, `LR1121Interface` - Semtech LR11xx chips
- `STM32WLE5JCInterface` - STM32WL integrated radio

**Routing**: Three routing strategies in the mesh:
- `FloodingRouter` - Basic flooding for reliability
- `ReliableRouter` - Adds acknowledgments and retransmissions
- `NextHopRouter` - Optimized routing using next-hop information

**Modules System**: Extensible plugin architecture where each module:
- Handles specific message types (portnum)
- Can send/receive/process packets
- Manages its own state and configuration
- Located in `src/modules/`

**Power Management**: Finite state machine (`PowerFSM.cpp`) manages device power states:
- BOOT → ON → DARK → SCREEN_ON → LS/NB_WAIT → DARK_WAIT → SDS/LS → etc.
- Controlled by user interaction, radio activity, and power configuration

**Threading**: Uses FreeRTOS (ESP32) or custom implementation (other platforms)
- Threads defined in files like `*Thread.h`
- Concurrency primitives in `src/concurrency/`

### Configuration System

Configuration is managed through:
- Protocol buffers (in `src/mesh/generated/`)
- `NodeDB` class for persistence
- `userPrefs.jsonc` for development overrides
- Variant-specific `variant.h` files for hardware configuration

### Communication Layers

1. **Physical**: LoRa radio (RadioInterface implementations)
2. **Mesh**: Routing and packet handling (Router, MeshService)
3. **API**: External interfaces (PhoneAPI, WiFiServerAPI, WebServer)
4. **Modules**: Application logic (various modules in `src/modules/`)

## Protocol Buffers

Protocol buffer definitions are automatically generated from the upstream meshtastic protobufs repository.

**Regenerate protobufs** (requires nanopb 0.4.9):
```bash
bin/regen-protos.sh
```

Generated files are placed in `src/mesh/generated/`.

## Hardware Variants

The firmware supports 125+ hardware variants. Each variant is defined in `variants/<variant-name>/`:
- Pin configurations
- Display types and settings
- Radio configurations
- Power settings
- Feature flags

Common variants are in directories like `heltec_v3/`, `tbeam/`, `rak4631/`, etc.

## Development Workflow

1. Make changes to source code
2. Format code: `trunk fmt`
3. Build for target: `source ~/.profile && platformio run -e <environment>`
4. Test on hardware or simulator
5. Run static analysis if needed: `source ~/.profile && platformio check`

## Platform-Specific Notes

**ESP32**: Primary platform with most features
- Supports WiFi, Bluetooth, web server
- Use `ARCH_ESP32` preprocessor flag

**nRF52**: Low power platform
- Bluetooth-focused
- Use `ARCH_NRF52` preprocessor flag

**RP2040**: Raspberry Pi Pico platform
- Cost-effective option
- Use `ARCH_RP2040` preprocessor flag

**Portduino**: Linux simulation platform
- Useful for testing without hardware
- Use `ARCH_PORTDUINO` preprocessor flag

## Important Build Flags

Key preprocessor definitions used throughout the codebase:
- `MESHTASTIC_EXCLUDE_*` - Exclude features to reduce binary size
- `HAS_WIFI`, `HAS_ETHERNET`, `HAS_GPS` - Platform capabilities
- `RADIOLIB_EXCLUDE_*` - Exclude unused radio modules
- Platform flags: `ARCH_ESP32`, `ARCH_NRF52`, `ARCH_RP2040`, `ARCH_STM32WL`, `ARCH_PORTDUINO`

## Serial Monitor

View serial output during development:
```bash
source ~/.profile && platformio device monitor
```

Monitor speed is 115200 baud by default. ESP32 includes exception decoder.

## Branch-Specific Notes

### treeguard-module Branch

This branch includes a TreeGuard module for vibration detection and classification using an MPU6050 accelerometer.

**Location:** `src/modules/treeguard/`

**Known Issue - Deep Sleep Implementation:**
The TreeGuard module currently has a **non-functional deep sleep implementation** that needs to be properly integrated with the Meshtastic power management system. Current issues:

1. **Bypasses PowerFSM:** Calls `esp_deep_sleep_start()` directly instead of using `doDeepSleep()` (src/modules/treeguard/TreeGuardModule.cpp:165)
2. **No sleep coordination:** Doesn't use the observable pattern (`preflightSleep`, `notifyDeepSleep`) to coordinate with other subsystems
3. **Radio timing issue:** Uses crude `delay(4000)` to wait for radio transmission instead of proper coordination
4. **No NodeDB save:** Doesn't save NodeDB before deep sleep, risking data loss
5. **Missing peripheral shutdown:** Doesn't properly shut down GPS, screen, and other peripherals

**Required Fix:**
The module needs to be refactored to use the existing Meshtastic sleep infrastructure:
- Use `doDeepSleep()` from `src/sleep.cpp` instead of calling ESP-IDF directly
- Integrate with PowerFSM state machine for proper power state management
- Use observable pattern to coordinate with radio, GPS, and other subsystems
- Ensure all peripherals are properly shut down before sleep

See `docs/ESP32_DEEP_SLEEP_RESEARCH.md` for detailed documentation on the proper deep sleep implementation.
