# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-based IoT control system for industrial dual stepper motor management ("Corte de Bobinas" = Bobbin Cutting). Controls two synchronized stepper motors via a web interface with real-time position feedback, OTA updates, and persistent configuration.

## Build & Upload Commands

This project uses **PlatformIO**. Run commands from the project root.

```bash
# Build
platformio run

# Upload via USB (COM3)
platformio run --target upload

# Upload via OTA (WiFi, target IP: 192.168.1.186)
platformio run --env esp32dev-ota --target upload

# Upload filesystem (web UI to LittleFS)
platformio run --target uploadfs

# Monitor serial output
platformio device monitor

# Build + upload in one step
platformio run --target upload && platformio device monitor
```

## Architecture

### Key Files

| File | Role |
|------|------|
| `src/main.cpp` | Entry point: WiFi, HTTP REST API, WebSocket server, button handling |
| `src/motor_logic.cpp` | Motor state machines, hardware timer ISRs, homing sequences |
| `include/config.h` | GPIO pin assignments, WiFi credentials, step resolutions, state enums |
| `include/motor_logic.h` | Public API declarations for motor control |
| `data/index.html` | Single-page web UI served from LittleFS |
| `platformio.ini` | Build config, board: `esp32dev`, dependencies |

### Motor Control Architecture

Motor control is **interrupt-driven** using ESP32 hardware timers:
- Each motor has its own timer ISR that generates STEP pulses
- State machines track motor phases (IDLE, RUNNING, HOMING, HOMING_BACKOFF, HOMING_FINE, PAUSE_AT_END, WAIT_SYNC_*, etc.)
- Synchronization between motors is handled by `checkSync()` — each motor waits at WAIT_SYNC_END / WAIT_SYNC_START before proceeding
- Homing is three-phase: fast approach → backoff → slow fine search

**Step resolutions** (defined in `config.h`):
- Motor 1: 8128 steps/unit
- Motor 2: 667 steps/unit

### Web Interface & API

The async web server (`ESPAsyncWebServer`) handles REST endpoints and a WebSocket at `/ws` that streams position updates every 100ms.

REST endpoints follow the pattern `/action[motor]` — e.g., `/startboth`, `/stopboth`, `/homing`, `/homingmotor2`, `/speed1`, `/speed2`, `/moveby1`, `/moveby2`, `/movefast1`, `/stopfast1`, `/saveMem`, `/loadMem`, `/getsettings`.

HTTP handlers run on the AsyncTCP task, so motion endpoints only enqueue a `Command` (FreeRTOS queue in `main.cpp`); `loop()` executes them via the API in `motor_logic.h`. Emergency stop sets `emergencyStop` immediately and is applied after the queue so it always wins. Motion commands are ignored unless the motor is `IDLE` (START while running does nothing). Physical buttons: START (GPIO16), STOP (GPIO17), EMERGENCY (GPIO18), HOME both motors (GPIO19); START and HOME are ignored unless both motors are IDLE. Hold-to-move (`/movefast*`) needs `/keepfast*` every 250 ms or the firmware stops after 1 s.

The web UI (`data/index.html`) is pure HTML/CSS/JS — no build step needed. After editing it, re-upload the filesystem with `uploadfs`.

### Persistent Storage

- **Preferences** (NVS): stores motor speeds, distances, delay timings
- **Memory slots**: 3 user-saveable presets via `/saveMem` and `/loadMem`
- **LittleFS**: hosts the web UI files

### Connectivity

- WiFi credentials live in `include/secrets.h` (gitignored); copy `include/secrets.example.h` to create it
- OTA hostname and IP target are in `platformio.ini` (`[env:esp32dev-ota]`)
- WebSocket endpoint: `ws://<device-ip>/ws`
