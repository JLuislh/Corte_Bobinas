#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- PINES ---
#define STEP_PIN          14
// OJO: GPIO12 es pin de arranque (MTDI). Si está en HIGH al encender, el ESP32 puede no
// arrancar (selecciona flash a 1.8 V). Solución de hardware: mover DIR a otro GPIO libre
// (p. ej. 33) o fijar el voltaje de flash por eFuse. No se puede corregir por software.
#define DIR_PIN           12
#define LIMIT_SWITCH_PIN  13
#define ENABLE_PIN        25   // TODO: verificar GPIO correcto en el hardware
#define SLEEP2_PIN        32
#define STEP2_PIN         15
#define DIR2_PIN          27
#define LIMIT_SWITCH2_PIN 26
#define ENABLE2_PIN       4
#define START_BUTTON_PIN         16
#define STOP_BUTTON_PIN          17
#define EMERGENCY_STOP_PIN       18   // TODO: verificar GPIO correcto en el hardware
#define HOME_BUTTON_PIN          19   // Homing de ambos motores (NO a GND, pull-up interno)

// --- WIFI ---
// Credenciales en include/secrets.h (no versionado). Ver secrets.example.h
#include "secrets.h"

// --- RESOLUCIÓN ---
const float steps_per_unit_m1 = 8128.0;
const float steps_per_unit_m2 = 667.0;

// --- ESTADOS ---
enum State { IDLE, RUNNING, RETURNING_HOME, HOMING, HOMING_BACKOFF, HOMING_FINE, MANUAL_MOVE, MANUAL_FAST_MOVE, PAUSE_AT_END, PAUSE_AT_START, WAIT_SYNC_END, WAIT_SYNC_START };
enum State2 { IDLE2, RUNNING2, RETURNING_HOME2, HOMING2, HOMING_BACKOFF2, HOMING_FINE2, MANUAL_MOVE2, MANUAL_FAST_MOVE2, PAUSE_AT_END2, PAUSE_AT_START2, WAIT_SYNC_END2, WAIT_SYNC_START2 };

#endif