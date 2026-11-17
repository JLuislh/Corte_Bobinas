#include <Arduino.h>
#include "motor_logic.h"
#include "config.h"

// --- DEFINICIÓN REAL DE VARIABLES (Viven aquí en el .cpp) ---
hw_timer_t *stepTimer = NULL;
hw_timer_t *stepTimer2 = NULL;

volatile long currentPosition_steps = 0;
volatile long currentPosition2_steps = 0;
volatile long targetPosition_steps = 0;
volatile long targetPosition2_steps = 0;
volatile int motor_speed_us = 400;
volatile int motor2_speed_us = 800;

volatile State currentState = IDLE;
volatile State2 currentState2 = IDLE2;

volatile unsigned long delay_at_end_ms = 0;
volatile unsigned long delay_at_start_ms = 0;
volatile unsigned long delay_at_end2_ms = 0;
volatile unsigned long delay_at_start2_ms = 0;

volatile float autoDistance_inches = 0.4;
volatile float autoDistance2_inches = 0.4;
volatile bool goingForward = true;
volatile int motor1_direction_multiplier = 1; // 1 for forward, -1 for backward
volatile bool goingForward2 = true;
volatile int motor2_direction_multiplier = 1; // 1 for forward, -1 for backward

bool stepHigh = false;
bool step2High = false;
unsigned long pauseStartTime = 0;
unsigned long pauseStartTime2 = 0;

// Flag de paro de emergencia: se activa desde cualquier core y detiene las ISRs inmediatamente
volatile bool emergencyStop = false;

// Flag de backoff Motor 2: necesita ser variable de archivo para poder resetearse en homingMotor2()
static bool m2BackoffCleared = false;

// --- INTERRUPCIONES (IRAM_ATTR) ---

void IRAM_ATTR onStepTimer() {
    if (emergencyStop || currentState == IDLE) return;
    if (stepHigh) {
        digitalWrite(STEP_PIN, LOW);
        stepHigh = false;
        currentPosition_steps += motor1_direction_multiplier;
    } else {
        digitalWrite(STEP_PIN, HIGH);
        stepHigh = true;
    }
}

void IRAM_ATTR onStepTimer2() {
    if (emergencyStop || currentState2 == IDLE2) return;
    if (step2High) {
        digitalWrite(STEP2_PIN, LOW);
        step2High = false;
        currentPosition2_steps += motor2_direction_multiplier;
    } else {
        digitalWrite(STEP2_PIN, HIGH);
        step2High = true;
    }
}

// --- FUNCIONES DE ACTUALIZACIÓN DE TIMER (Sintaxis v2.0.14) ---

void updateStepTimer(bool enable) {
    if (!stepTimer) return;
    if (enable) {
        timerAlarmWrite(stepTimer, motor_speed_us, true);
        timerAlarmEnable(stepTimer);
    } else {
        timerAlarmDisable(stepTimer);
        digitalWrite(STEP_PIN, LOW);
    }
}

void updateStepTimer2(bool enable) {
    if (!stepTimer2) return;
    if (enable) {
        timerAlarmWrite(stepTimer2, motor2_speed_us, true);
        timerAlarmEnable(stepTimer2);
    } else {
        timerAlarmDisable(stepTimer2);
        digitalWrite(STEP2_PIN, LOW);
    }
}

// --- LÓGICA DE CONTROL ---

void startBothMotors() {
    emergencyStop = false; // limpiar paro de emergencia antes de arrancar
    currentPosition_steps = 0;
    targetPosition_steps = (long)(autoDistance_inches * steps_per_unit_m1);
    goingForward = true;
    motor1_direction_multiplier = 1;
    currentState = RUNNING;
    updateStepTimer(true);

    currentPosition2_steps = 0;
    targetPosition2_steps = (long)(autoDistance2_inches * steps_per_unit_m2);
    goingForward2 = true;
    motor2_direction_multiplier = 1;
    currentState2 = RUNNING2;
    updateStepTimer2(true);
}

void stopBothMotors() {
    targetPosition_steps = 0;
    currentState = RETURNING_HOME;
    motor1_direction_multiplier = -1;
    updateStepTimer(true);
    
    targetPosition2_steps = 0;
    currentState2 = RETURNING_HOME2;
    motor2_direction_multiplier = -1;
    updateStepTimer2(true);
}

void handleMotor() {
    static State lastState = IDLE; // Keep track of the previous state
    static int normalSpeed_m1; // Para guardar la velocidad antes del homing fino
    if (currentState != lastState) {
        if (currentState == IDLE) {
            digitalWrite(ENABLE_PIN, HIGH); // Disable motor
            updateStepTimer(false); // Ensure timer is off
        } else {
            digitalWrite(ENABLE_PIN, LOW); // Enable motor
        }
        lastState = currentState;
    }

    if (currentState == IDLE) {
        return;
    }
    unsigned long now_us = micros();
    // Sensor NC (normalmente cerrado): pin LOW = libre, pin HIGH = sensor activado
    bool homeSwitch = (digitalRead(LIMIT_SWITCH_PIN) == HIGH);
    
    // --- HOMING DE TRES FASES PRECISO ---
    if (currentState == HOMING) {
        // Fase 1: Ir hacia el sensor rápido (DIR HIGH)
        motor1_direction_multiplier = -1;
        digitalWrite(DIR_PIN, HIGH);
        if (homeSwitch) {
            currentPosition_steps = 0;
            updateStepTimer(false);
            currentState = HOMING_BACKOFF;
            goingForward = false;
            updateStepTimer(true);
        }
        return;
    }

    if (currentState == HOMING_BACKOFF) {
        // Fase 2: Alejarse del sensor (DIR LOW) hasta que se libere + margen extra
        static bool m1BackoffCleared = false;
        motor1_direction_multiplier = 1;
        digitalWrite(DIR_PIN, LOW);
        if (!m1BackoffCleared) {
            if (!homeSwitch) {          // esperar liberación física del sensor
                m1BackoffCleared = true;
                currentPosition_steps = 0;
                targetPosition_steps = 500; // pasos extra de margen
            }
        } else if (currentPosition_steps >= targetPosition_steps) {
            m1BackoffCleared = false;
            currentPosition_steps = 0;
            updateStepTimer(false);
            normalSpeed_m1 = motor_speed_us;
            motor_speed_us = normalSpeed_m1 * 2; // mitad de velocidad para fase fina
            currentState = HOMING_FINE;
            goingForward = true;
            updateStepTimer(true);
        }
        return;
    }

    if (currentState == HOMING_FINE) {
        // Fase 3: Volver lentamente hacia el sensor (DIR HIGH, precisión)
        motor1_direction_multiplier = -1;
        digitalWrite(DIR_PIN, HIGH);
        if (homeSwitch) {
            currentPosition_steps = 0;
            updateStepTimer(false);
            motor_speed_us = normalSpeed_m1;
            currentState = IDLE;
            Serial.println("Motor 1 homing preciso completado");
        }
        return;
    }
    
    if (currentState == PAUSE_AT_END || currentState == PAUSE_AT_START) {
        updateStepTimer(false);
        unsigned long pauseDuration = (currentState == PAUSE_AT_END) ? delay_at_end_ms : delay_at_start_ms;
        if ((now_us - pauseStartTime) >= (pauseDuration * 1000UL)) {
            goingForward = (currentState == PAUSE_AT_END) ? false : true;
            if (currentState == PAUSE_AT_START) {
                // Recalcular target con la distancia actual (permite cambiar en caliente)
                currentPosition_steps = 0;
                targetPosition_steps = (long)(autoDistance_inches * steps_per_unit_m1);
            }
            currentState = RUNNING;
            updateStepTimer(true);
        }
        return;
    }
    
    if (currentState == RUNNING) {
        if (goingForward) {
            motor1_direction_multiplier = 1;
            digitalWrite(DIR_PIN, LOW);
            if (currentPosition_steps >= targetPosition_steps) {
                updateStepTimer(false);
                currentState = WAIT_SYNC_END;
            }
        } else {
            motor1_direction_multiplier = -1;
            digitalWrite(DIR_PIN, HIGH);
            if (currentPosition_steps <= 0) {
                updateStepTimer(false);
                currentState = WAIT_SYNC_START;
            }
        }
    } else if (currentState == MANUAL_MOVE) {
        if (goingForward) {
            motor1_direction_multiplier = 1;
            digitalWrite(DIR_PIN, LOW);
            if (currentPosition_steps >= targetPosition_steps) {
                updateStepTimer(false);
                currentState = IDLE;
            }
        } else {
            motor1_direction_multiplier = -1;
            digitalWrite(DIR_PIN, HIGH);
            if (currentPosition_steps <= 0) {
                updateStepTimer(false);
                currentState = IDLE;
            }
        }
    } else if (currentState == MANUAL_FAST_MOVE) {
        // Movimiento rápido/continuo sin límite (solo dirección)
        if (goingForward) {
            digitalWrite(DIR_PIN, LOW);
        } else {
            digitalWrite(DIR_PIN, HIGH);
        }
    } else if (currentState == RETURNING_HOME) {
        // Regresa a home usando comparación de posición
        if (currentPosition_steps > targetPosition_steps) {
            motor1_direction_multiplier = -1;
            digitalWrite(DIR_PIN, HIGH);
        } else if (currentPosition_steps < targetPosition_steps) {
            motor1_direction_multiplier = 1;
            digitalWrite(DIR_PIN, LOW);
        } else {
            updateStepTimer(false);
            currentState = IDLE;
        }
    }
}

void homingMotor2() {
    emergencyStop = false;
    m2BackoffCleared = false; // Resetear flag para nuevo ciclo de homing
    currentPosition2_steps = 0;
    targetPosition2_steps = 0; // Target for homing is 0
    goingForward2 = false;
    currentState2 = HOMING2;
    updateStepTimer2(true);
}

void handleMotor2() {
    static State2 lastState2 = IDLE2; // Keep track of the previous state
    static int normalSpeed_m2; // Para guardar la velocidad antes del homing fino
    if (currentState2 != lastState2) {
        if (currentState2 == IDLE2) {
            digitalWrite(ENABLE2_PIN, HIGH); // Disable motor
            digitalWrite(SLEEP2_PIN, LOW);   // Put driver to sleep
            updateStepTimer2(false); // Ensure timer is off
        } else {
            digitalWrite(ENABLE2_PIN, LOW);  // Enable motor
            digitalWrite(SLEEP2_PIN, HIGH);  // Wake up driver
        }
        lastState2 = currentState2;
    }

    if (currentState2 == IDLE2) {
        return; // Si está en IDLE, no hace nada más
    }
    
    unsigned long now_us = micros();
    // Sensor NO (normalmente abierto): pin HIGH = libre, pin LOW = sensor activado
    bool homeSwitch = (digitalRead(LIMIT_SWITCH2_PIN) == LOW);
    
    // --- HOMING DE TRES FASES PRECISO PARA MOTOR 2 ---
    if (currentState2 == HOMING2) {
        // Fase 1: Ir hacia el sensor rápido
        static unsigned long lastHomingPrint = 0;
        if (millis() - lastHomingPrint > 500) {
            lastHomingPrint = millis();
            Serial.printf("[HOMING2] homeSwitch=%d  pos=%ld\n", homeSwitch, currentPosition2_steps);
        }
        motor2_direction_multiplier = -1;
        digitalWrite(DIR2_PIN, LOW);
        if (homeSwitch) {
            currentPosition2_steps = 0;
            updateStepTimer2(false);
            currentState2 = HOMING_BACKOFF2;
            goingForward2 = false;
            updateStepTimer2(true);
        }
        return;
    }

    if (currentState2 == HOMING_BACKOFF2) {
        // Fase 2: Alejarse del sensor (DIR LOW) hasta liberarse + margen extra
        motor2_direction_multiplier = 1;
        digitalWrite(DIR2_PIN, HIGH);
        if (!m2BackoffCleared) {
            if (!homeSwitch) {
                m2BackoffCleared = true;
                currentPosition2_steps = 0;
                targetPosition2_steps = 300;
            }
        } else if (currentPosition2_steps >= targetPosition2_steps) {
            m2BackoffCleared = false;
            currentPosition2_steps = 0;
            updateStepTimer2(false);
            normalSpeed_m2 = motor2_speed_us;
            motor2_speed_us = normalSpeed_m2 * 2;
            currentState2 = HOMING_FINE2;
            goingForward2 = false;
            updateStepTimer2(true);
        }
        return;
    }

    if (currentState2 == HOMING_FINE2) {
        // Fase 3: Volver lentamente hacia el sensor (DIR HIGH, precisión)
        motor2_direction_multiplier = -1;
        digitalWrite(DIR2_PIN, LOW);
        if (homeSwitch) {
            currentPosition2_steps = 0;
            updateStepTimer2(false);
            motor2_speed_us = normalSpeed_m2;
            currentState2 = IDLE2;
            Serial.println("Motor 2 homing preciso completado");
        }
        return;
    }
    
    if (currentState2 == PAUSE_AT_END2 || currentState2 == PAUSE_AT_START2) {
        updateStepTimer2(false);
        unsigned long pauseDuration = (currentState2 == PAUSE_AT_END2) ? delay_at_end2_ms : delay_at_start2_ms;
        if ((now_us - pauseStartTime2) >= (pauseDuration * 1000UL)) {
            goingForward2 = (currentState2 == PAUSE_AT_END2) ? false : true;
            if (currentState2 == PAUSE_AT_START2) {
                // Recalcular target con la distancia actual (permite cambiar en caliente)
                currentPosition2_steps = 0;
                targetPosition2_steps = (long)(autoDistance2_inches * steps_per_unit_m2);
            }
            currentState2 = RUNNING2;
            updateStepTimer2(true);
        }
        return;
    }
    
    if (currentState2 == RUNNING2) {
        if (goingForward2) {
            motor2_direction_multiplier = 1;
            digitalWrite(DIR2_PIN, HIGH);   // igual que Motor 1
            if (currentPosition2_steps >= targetPosition2_steps) {
                updateStepTimer2(false);
                currentState2 = WAIT_SYNC_END2;
            }
        } else {
            motor2_direction_multiplier = -1;
            digitalWrite(DIR2_PIN, LOW);  // igual que Motor 1
            if (currentPosition2_steps <= 0) {
                updateStepTimer2(false);
                currentState2 = WAIT_SYNC_START2;
            }
        }
    } else if (currentState2 == MANUAL_MOVE2) {
        if (goingForward2) {
            motor2_direction_multiplier = 1;
            digitalWrite(DIR2_PIN, HIGH);
            if (currentPosition2_steps >= targetPosition2_steps) {
                updateStepTimer2(false);
                currentState2 = IDLE2;
            }
        } else {
            motor2_direction_multiplier = -1;
            digitalWrite(DIR2_PIN, LOW);
            if (currentPosition2_steps <= 0) {
                updateStepTimer2(false);
                currentState2 = IDLE2;
            }
        }
    } else if (currentState2 == MANUAL_FAST_MOVE2) {
        if (goingForward2) {
            digitalWrite(DIR2_PIN, HIGH);
        } else {
            digitalWrite(DIR2_PIN, LOW);
        }
    } else if (currentState2 == RETURNING_HOME2) {
        if (currentPosition2_steps > targetPosition2_steps) {
            motor2_direction_multiplier = -1;
            digitalWrite(DIR2_PIN, LOW);  // hacia home, igual que Motor 1
        } else if (currentPosition2_steps < targetPosition2_steps) {
            motor2_direction_multiplier = 1;
            digitalWrite(DIR2_PIN, HIGH);
        } else {
            updateStepTimer2(false);
            currentState2 = IDLE2;
        }
    }
}

void checkSync() {
    if (currentState == WAIT_SYNC_END && currentState2 == WAIT_SYNC_END2) {
        currentState = PAUSE_AT_END;
        pauseStartTime = micros();
        currentState2 = PAUSE_AT_END2;
        pauseStartTime2 = micros();
    }
    if (currentState == WAIT_SYNC_START && currentState2 == WAIT_SYNC_START2) {
        currentState = PAUSE_AT_START;
        pauseStartTime = micros();
        currentState2 = PAUSE_AT_START2;
        pauseStartTime2 = micros();
    }
}