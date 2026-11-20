#include <Arduino.h>
#include "motor_logic.h"
#include "config.h"

// --- DEFINICIÓN REAL DE VARIABLES ---
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
volatile int motor1_direction_multiplier = 1;
volatile bool goingForward2 = true;
volatile int motor2_direction_multiplier = 1;

static volatile bool stepHigh = false;
static volatile bool step2High = false;
static unsigned long pauseStartTime = 0;
static unsigned long pauseStartTime2 = 0;

// Paro de emergencia: las ISRs dejan de generar pulsos en cuanto se activa
volatile bool emergencyStop = false;

// Flags de la fase de backoff del homing
static bool m1BackoffCleared = false;
static bool m2BackoffCleared = false;

// Watchdog del movimiento continuo: si la web deja de mandar keep-alive, el motor se detiene
static volatile unsigned long lastFastKeep1 = 0;
static volatile unsigned long lastFastKeep2 = 0;
static const unsigned long FAST_MOVE_TIMEOUT_MS = 1000;

static const long HOMING_BACKOFF_STEPS_M1 = 500;
static const long HOMING_BACKOFF_STEPS_M2 = 300;

// --- INTERRUPCIONES ---
// Cada disparo del timer alterna STEP: un paso completo = 2 disparos (2 × speed_us)

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

// --- HELPERS DE HARDWARE ---

// El homing fino va a media velocidad sin tocar la velocidad configurada
static uint64_t stepPeriod1() {
    return (currentState == HOMING_FINE) ? (uint64_t)motor_speed_us * 2 : (uint64_t)motor_speed_us;
}
static uint64_t stepPeriod2() {
    return (currentState2 == HOMING_FINE2) ? (uint64_t)motor2_speed_us * 2 : (uint64_t)motor2_speed_us;
}

// Motor 1: adelante = DIR LOW
static void setDir1(bool forward) {
    goingForward = forward;
    motor1_direction_multiplier = forward ? 1 : -1;
    digitalWrite(DIR_PIN, forward ? LOW : HIGH);
}
// Motor 2: adelante = DIR HIGH
static void setDir2(bool forward) {
    goingForward2 = forward;
    motor2_direction_multiplier = forward ? 1 : -1;
    digitalWrite(DIR2_PIN, forward ? HIGH : LOW);
}

static void enable1(bool on) {
    digitalWrite(ENABLE_PIN, on ? LOW : HIGH);
}
static void enable2(bool on) {
    digitalWrite(ENABLE2_PIN, on ? LOW : HIGH);
    digitalWrite(SLEEP2_PIN, on ? HIGH : LOW);
}

void updateStepTimer(bool enable) {
    if (!stepTimer) return;
    timerAlarmDisable(stepTimer);
    if (enable) {
        timerWrite(stepTimer, 0);  // evita que el contador quede por encima de una alarma nueva
        timerAlarmWrite(stepTimer, stepPeriod1(), true);
        timerAlarmEnable(stepTimer);
    } else {
        // Si quedó un pulso a medias, el driver ya vio el flanco de subida: contarlo
        if (stepHigh) {
            stepHigh = false;
            currentPosition_steps += motor1_direction_multiplier;
        }
        digitalWrite(STEP_PIN, LOW);
    }
}

void updateStepTimer2(bool enable) {
    if (!stepTimer2) return;
    timerAlarmDisable(stepTimer2);
    if (enable) {
        timerWrite(stepTimer2, 0);
        timerAlarmWrite(stepTimer2, stepPeriod2(), true);
        timerAlarmEnable(stepTimer2);
    } else {
        if (step2High) {
            step2High = false;
            currentPosition2_steps += motor2_direction_multiplier;
        }
        digitalWrite(STEP2_PIN, LOW);
    }
}

// Detiene el timer (contabilizando el pulso pendiente con la dirección anterior),
// fija la nueva dirección y arranca en el estado indicado
static void beginMotion1(State s, bool forward) {
    updateStepTimer(false);
    setDir1(forward);
    currentState = s;
    enable1(true);
    updateStepTimer(true);
}
static void beginMotion2(State2 s, bool forward) {
    updateStepTimer2(false);
    setDir2(forward);
    currentState2 = s;
    enable2(true);
    updateStepTimer2(true);
}

static void haltMotor1() {
    updateStepTimer(false);
    currentState = IDLE;
}
static void haltMotor2() {
    updateStepTimer2(false);
    currentState2 = IDLE2;
}

static bool isHoming1() {
    return currentState == HOMING || currentState == HOMING_BACKOFF || currentState == HOMING_FINE;
}
static bool isHoming2() {
    return currentState2 == HOMING2 || currentState2 == HOMING_BACKOFF2 || currentState2 == HOMING_FINE2;
}
static bool inCycle1() {
    return currentState == RUNNING || currentState == WAIT_SYNC_END || currentState == WAIT_SYNC_START ||
           currentState == PAUSE_AT_END || currentState == PAUSE_AT_START;
}
static bool inCycle2() {
    return currentState2 == RUNNING2 || currentState2 == WAIT_SYNC_END2 || currentState2 == WAIT_SYNC_START2 ||
           currentState2 == PAUSE_AT_END2 || currentState2 == PAUSE_AT_START2;
}

// --- COMANDOS ---

static void startCycle1() {
    currentPosition_steps = 0;
    targetPosition_steps = (long)(autoDistance_inches * steps_per_unit_m1);
    beginMotion1(RUNNING, true);
}
static void startCycle2() {
    currentPosition2_steps = 0;
    targetPosition2_steps = (long)(autoDistance2_inches * steps_per_unit_m2);
    beginMotion2(RUNNING2, true);
}

bool startBothMotors() {
    // START con cualquier motor en movimiento no hace nada
    if (currentState != IDLE || currentState2 != IDLE2) return false;
    emergencyStop = false;
    startCycle1();
    startCycle2();
    return true;
}

bool startAuto1() {
    if (currentState != IDLE) return false;
    emergencyStop = false;
    startCycle1();
    return true;
}
bool startAuto2() {
    if (currentState2 != IDLE2) return false;
    emergencyStop = false;
    startCycle2();
    return true;
}

// Regresa al origen del ciclo; si estaba haciendo homing, se detiene en el lugar
static void returnHome1() {
    if (isHoming1()) { haltMotor1(); return; }
    targetPosition_steps = 0;
    if (currentPosition_steps == 0) { haltMotor1(); return; }
    beginMotion1(RETURNING_HOME, currentPosition_steps < 0);
}
static void returnHome2() {
    if (isHoming2()) { haltMotor2(); return; }
    targetPosition2_steps = 0;
    if (currentPosition2_steps == 0) { haltMotor2(); return; }
    beginMotion2(RETURNING_HOME2, currentPosition2_steps < 0);
}

void stopBothMotors() {
    emergencyStop = false;
    returnHome1();
    returnHome2();
}
void stopAuto1() {
    emergencyStop = false;
    returnHome1();
}
void stopAuto2() {
    emergencyStop = false;
    returnHome2();
}

void emergencyStopAll() {
    emergencyStop = true;   // primero: las ISRs dejan de pulsar de inmediato
    updateStepTimer(false);
    updateStepTimer2(false);
    currentState  = IDLE;
    currentState2 = IDLE2;
    m1BackoffCleared = false;
    m2BackoffCleared = false;
    enable1(false);
    enable2(false);
}

bool homingMotor1() {
    if (currentState != IDLE) return false;
    emergencyStop = false;
    m1BackoffCleared = false;
    currentPosition_steps = 0;
    targetPosition_steps = 0;
    beginMotion1(HOMING, false);
    return true;
}
bool homingMotor2() {
    if (currentState2 != IDLE2) return false;
    emergencyStop = false;
    m2BackoffCleared = false;
    currentPosition2_steps = 0;
    targetPosition2_steps = 0;
    beginMotion2(HOMING2, false);
    return true;
}

bool homeBothMotors() {
    // Igual que START: con cualquier motor en movimiento no hace nada
    if (currentState != IDLE || currentState2 != IDLE2) return false;
    homingMotor1();
    homingMotor2();
    return true;
}

bool moveBy1(bool forward, float dist) {
    if (currentState != IDLE) return false;
    long steps = (long)(dist * steps_per_unit_m1);
    if (steps <= 0) return false;
    emergencyStop = false;
    targetPosition_steps = currentPosition_steps + (forward ? steps : -steps);
    beginMotion1(MANUAL_MOVE, forward);
    return true;
}
bool moveBy2(bool forward, float dist) {
    if (currentState2 != IDLE2) return false;
    long steps = (long)(dist * steps_per_unit_m2);
    if (steps <= 0) return false;
    emergencyStop = false;
    targetPosition2_steps = currentPosition2_steps + (forward ? steps : -steps);
    beginMotion2(MANUAL_MOVE2, forward);
    return true;
}

bool fastMove1(bool forward) {
    if (currentState != IDLE && currentState != MANUAL_FAST_MOVE) return false;
    emergencyStop = false;
    lastFastKeep1 = millis();
    if (currentState == MANUAL_FAST_MOVE && goingForward == forward) return true;
    beginMotion1(MANUAL_FAST_MOVE, forward);
    return true;
}
bool fastMove2(bool forward) {
    if (currentState2 != IDLE2 && currentState2 != MANUAL_FAST_MOVE2) return false;
    emergencyStop = false;
    lastFastKeep2 = millis();
    if (currentState2 == MANUAL_FAST_MOVE2 && goingForward2 == forward) return true;
    beginMotion2(MANUAL_FAST_MOVE2, forward);
    return true;
}

// Solo detiene un movimiento continuo; no afecta ciclos ni homing
void stopFast1() {
    if (currentState == MANUAL_FAST_MOVE) haltMotor1();
}
void stopFast2() {
    if (currentState2 == MANUAL_FAST_MOVE2) haltMotor2();
}

void keepFast1() { lastFastKeep1 = millis(); }
void keepFast2() { lastFastKeep2 = millis(); }

void setSpeed1(int us) {
    if (us < 50) return;
    motor_speed_us = us;
    if (stepTimer && timerAlarmEnabled(stepTimer)) updateStepTimer(true);
}
void setSpeed2(int us) {
    if (us < 50) return;
    motor2_speed_us = us;
    if (stepTimer2 && timerAlarmEnabled(stepTimer2)) updateStepTimer2(true);
}

// --- LÓGICA DE CONTROL ---

void handleMotor() {
    static State lastState = IDLE;
    if (currentState != lastState) {
        if (currentState == IDLE) {
            updateStepTimer(false);
            enable1(false);
        } else {
            enable1(true);
        }
        lastState = currentState;
    }
    if (currentState == IDLE) return;

    // Sensor NC (normalmente cerrado): pin LOW = libre, pin HIGH = sensor activado
    bool homeSwitch = (digitalRead(LIMIT_SWITCH_PIN) == HIGH);

    switch (currentState) {
    case HOMING:  // Fase 1: rápido hacia el sensor
        if (homeSwitch) {
            updateStepTimer(false);
            currentPosition_steps = 0;
            m1BackoffCleared = false;
            currentState = HOMING_BACKOFF;
            setDir1(true);
            updateStepTimer(true);
        }
        break;

    case HOMING_BACKOFF:  // Fase 2: alejarse hasta liberar el sensor + margen
        if (!m1BackoffCleared) {
            if (!homeSwitch) {
                m1BackoffCleared = true;
                currentPosition_steps = 0;
                targetPosition_steps = HOMING_BACKOFF_STEPS_M1;
            }
        } else if (currentPosition_steps >= targetPosition_steps) {
            updateStepTimer(false);
            m1BackoffCleared = false;
            currentPosition_steps = 0;
            currentState = HOMING_FINE;  // stepPeriod1() aplica media velocidad
            setDir1(false);
            updateStepTimer(true);
        }
        break;

    case HOMING_FINE:  // Fase 3: lento hacia el sensor
        if (homeSwitch) {
            haltMotor1();
            currentPosition_steps = 0;
            Serial.println("Motor 1 homing preciso completado");
        }
        break;

    case PAUSE_AT_END:
    case PAUSE_AT_START: {
        unsigned long dur = (currentState == PAUSE_AT_END) ? delay_at_end_ms : delay_at_start_ms;
        if ((micros() - pauseStartTime) >= dur * 1000UL) {
            bool forward = (currentState == PAUSE_AT_START);
            if (forward) {
                // Recalcular con la distancia actual (permite cambiarla en caliente)
                currentPosition_steps = 0;
                targetPosition_steps = (long)(autoDistance_inches * steps_per_unit_m1);
            }
            setDir1(forward);
            currentState = RUNNING;
            updateStepTimer(true);
        }
        break;
    }

    case RUNNING:
        if (goingForward) {
            if (currentPosition_steps >= targetPosition_steps) {
                updateStepTimer(false);
                currentState = WAIT_SYNC_END;
            }
        } else if (currentPosition_steps <= 0) {
            updateStepTimer(false);
            currentState = WAIT_SYNC_START;
        }
        break;

    case MANUAL_MOVE:
    case RETURNING_HOME:
        if (goingForward ? (currentPosition_steps >= targetPosition_steps)
                         : (currentPosition_steps <= targetPosition_steps)) {
            haltMotor1();
        }
        break;

    case MANUAL_FAST_MOVE:
        if (millis() - lastFastKeep1 > FAST_MOVE_TIMEOUT_MS) {
            haltMotor1();
            Serial.println("Motor 1: movimiento continuo detenido (sin keep-alive)");
        }
        break;

    default:  // WAIT_SYNC_*: lo resuelve checkSync()
        break;
    }
}

void handleMotor2() {
    static State2 lastState2 = IDLE2;
    if (currentState2 != lastState2) {
        if (currentState2 == IDLE2) {
            updateStepTimer2(false);
            enable2(false);
        } else {
            enable2(true);
        }
        lastState2 = currentState2;
    }
    if (currentState2 == IDLE2) return;

    // Sensor NO (normalmente abierto): pin HIGH = libre, pin LOW = sensor activado
    bool homeSwitch = (digitalRead(LIMIT_SWITCH2_PIN) == LOW);

    switch (currentState2) {
    case HOMING2:
        if (homeSwitch) {
            updateStepTimer2(false);
            currentPosition2_steps = 0;
            m2BackoffCleared = false;
            currentState2 = HOMING_BACKOFF2;
            setDir2(true);
            updateStepTimer2(true);
        }
        break;

    case HOMING_BACKOFF2:
        if (!m2BackoffCleared) {
            if (!homeSwitch) {
                m2BackoffCleared = true;
                currentPosition2_steps = 0;
                targetPosition2_steps = HOMING_BACKOFF_STEPS_M2;
            }
        } else if (currentPosition2_steps >= targetPosition2_steps) {
            updateStepTimer2(false);
            m2BackoffCleared = false;
            currentPosition2_steps = 0;
            currentState2 = HOMING_FINE2;
            setDir2(false);
            updateStepTimer2(true);
        }
        break;

    case HOMING_FINE2:
        if (homeSwitch) {
            haltMotor2();
            currentPosition2_steps = 0;
            Serial.println("Motor 2 homing preciso completado");
        }
        break;

    case PAUSE_AT_END2:
    case PAUSE_AT_START2: {
        unsigned long dur = (currentState2 == PAUSE_AT_END2) ? delay_at_end2_ms : delay_at_start2_ms;
        if ((micros() - pauseStartTime2) >= dur * 1000UL) {
            bool forward = (currentState2 == PAUSE_AT_START2);
            if (forward) {
                currentPosition2_steps = 0;
                targetPosition2_steps = (long)(autoDistance2_inches * steps_per_unit_m2);
            }
            setDir2(forward);
            currentState2 = RUNNING2;
            updateStepTimer2(true);
        }
        break;
    }

    case RUNNING2:
        if (goingForward2) {
            if (currentPosition2_steps >= targetPosition2_steps) {
                updateStepTimer2(false);
                currentState2 = WAIT_SYNC_END2;
            }
        } else if (currentPosition2_steps <= 0) {
            updateStepTimer2(false);
            currentState2 = WAIT_SYNC_START2;
        }
        break;

    case MANUAL_MOVE2:
    case RETURNING_HOME2:
        if (goingForward2 ? (currentPosition2_steps >= targetPosition2_steps)
                          : (currentPosition2_steps <= targetPosition2_steps)) {
            haltMotor2();
        }
        break;

    case MANUAL_FAST_MOVE2:
        if (millis() - lastFastKeep2 > FAST_MOVE_TIMEOUT_MS) {
            haltMotor2();
            Serial.println("Motor 2: movimiento continuo detenido (sin keep-alive)");
        }
        break;

    default:
        break;
    }
}

// Sincroniza los extremos del ciclo. Si el otro motor no está en ciclo
// (p. ej. /startauto1 solo), el motor continúa sin esperarlo.
void checkSync() {
    bool c1 = inCycle1(), c2 = inCycle2();
    bool end1 = (currentState == WAIT_SYNC_END),     end2 = (currentState2 == WAIT_SYNC_END2);
    bool start1 = (currentState == WAIT_SYNC_START), start2 = (currentState2 == WAIT_SYNC_START2);
    unsigned long now = micros();

    if (end1 && (end2 || !c2))     { currentState = PAUSE_AT_END;    pauseStartTime = now; }
    if (end2 && (end1 || !c1))     { currentState2 = PAUSE_AT_END2;  pauseStartTime2 = now; }
    if (start1 && (start2 || !c2)) { currentState = PAUSE_AT_START;   pauseStartTime = now; }
    if (start2 && (start1 || !c1)) { currentState2 = PAUSE_AT_START2; pauseStartTime2 = now; }
}
