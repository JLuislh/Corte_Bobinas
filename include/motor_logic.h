#ifndef MOTOR_LOGIC_H
#define MOTOR_LOGIC_H

#include <Arduino.h>
#include <esp_timer.h>
//#include <driver/gptimer.h>
#include "config.h"

// Declaración de funciones
void IRAM_ATTR onStepTimer();
void IRAM_ATTR onStepTimer2();
void updateStepTimer(bool enable);
void updateStepTimer2(bool enable);
void handleMotor();
void handleMotor2();
extern volatile int motor1_direction_multiplier;
extern volatile int motor2_direction_multiplier;
extern volatile bool emergencyStop;
void homingMotor2();
void checkSync();
void startBothMotors();
void stopBothMotors();

#endif