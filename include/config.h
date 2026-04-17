#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- PINES ---
#define STEP_PIN          14
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

// --- WIFI ---
#define WIFI_SSID "INGENIERIA"
#define WIFI_PW   "Elettronici17"

// --- RESOLUCIÓN ---
const float steps_per_unit_m1 = 8128.0;
const float steps_per_unit_m2 = 1016.0;

// --- ESTADOS ---
enum State { IDLE, RUNNING, RETURNING_HOME, HOMING, HOMING_BACKOFF, HOMING_FINE, MANUAL_MOVE, MANUAL_FAST_MOVE, PAUSE_AT_END, PAUSE_AT_START, WAIT_SYNC_END, WAIT_SYNC_START };
enum State2 { IDLE2, RUNNING2, RETURNING_HOME2, HOMING2, HOMING_BACKOFF2, HOMING_FINE2, MANUAL_MOVE2, MANUAL_FAST_MOVE2, PAUSE_AT_END2, PAUSE_AT_START2, WAIT_SYNC_END2, WAIT_SYNC_START2 };

#endif