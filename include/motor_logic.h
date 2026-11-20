#ifndef MOTOR_LOGIC_H
#define MOTOR_LOGIC_H

#include <Arduino.h>
#include "config.h"

// --- ESTADO COMPARTIDO (definido en motor_logic.cpp) ---
extern hw_timer_t *stepTimer;
extern hw_timer_t *stepTimer2;

extern volatile long currentPosition_steps;
extern volatile long currentPosition2_steps;
extern volatile long targetPosition_steps;
extern volatile long targetPosition2_steps;
extern volatile int motor_speed_us;     // medio periodo del pulso STEP (µs)
extern volatile int motor2_speed_us;

extern volatile State currentState;
extern volatile State2 currentState2;

extern volatile bool goingForward;
extern volatile bool goingForward2;
extern volatile int motor1_direction_multiplier;
extern volatile int motor2_direction_multiplier;

extern volatile unsigned long delay_at_end_ms;
extern volatile unsigned long delay_at_start_ms;
extern volatile unsigned long delay_at_end2_ms;
extern volatile unsigned long delay_at_start2_ms;

extern volatile float autoDistance_inches;
extern volatile float autoDistance2_inches;

extern volatile bool emergencyStop;

// --- ISR / TIMERS ---
void IRAM_ATTR onStepTimer();
void IRAM_ATTR onStepTimer2();
void updateStepTimer(bool enable);
void updateStepTimer2(bool enable);

// --- LAZO PRINCIPAL (llamar solo desde loop()) ---
void handleMotor();
void handleMotor2();
void checkSync();

// --- COMANDOS (llamar solo desde loop(); devuelven false si se ignoraron) ---
bool startBothMotors();          // solo arranca si ambos motores están en IDLE
void stopBothMotors();           // regresa a origen (o detiene el homing en curso)
void emergencyStopAll();
bool homingMotor1();
bool homingMotor2();
bool homeBothMotors();           // solo arranca si ambos motores están en IDLE
bool startAuto1();
bool startAuto2();
void stopAuto1();
void stopAuto2();
bool moveBy1(bool forward, float dist);
bool moveBy2(bool forward, float dist);
bool fastMove1(bool forward);
bool fastMove2(bool forward);
void stopFast1();
void stopFast2();
void setSpeed1(int us);
void setSpeed2(int us);

// Keep-alive del movimiento continuo (seguro llamarlo desde cualquier tarea)
void keepFast1();
void keepFast2();

#endif
