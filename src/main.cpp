#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <AsyncWebSocket.h>
#include <Preferences.h>
#include <ArduinoOTA.h>

// Importamos nuestras particiones de código
#include "config.h"
#include "motor_logic.h"

// --- DECLARACIONES EXTERN (PARA QUE EL MAIN RECONOZCA LO QUE ESTÁ EN MOTOR_LOGIC.H/CPP) ---
extern hw_timer_t *stepTimer;
extern hw_timer_t *stepTimer2;

extern volatile long currentPosition_steps;
extern volatile long currentPosition2_steps;
extern volatile long targetPosition_steps;
extern volatile long targetPosition2_steps;
extern volatile int motor_speed_us;
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

extern void updateStepTimer(bool enable);
extern void updateStepTimer2(bool enable);
extern void homingMotor2();
extern void startBothMotors();
extern void stopBothMotors();
extern void handleMotor();
extern void handleMotor2();
extern void checkSync();

// Instancias Globales
AsyncWebServer server(80);
Preferences pref;
AsyncWebSocket ws("/ws"); // Declaración del servidor WebSocket en el endpoint /ws

// Función para enviar las posiciones de los motores a todos los clientes WebSocket conectados
void sendMotorPositions() {
    if (ws.count() == 0) return; // No hay clientes conectados, no hacer nada

    JsonDocument doc;
    doc["pos1"] = (float)currentPosition_steps / steps_per_unit_m1;
    doc["pos2"] = (float)currentPosition2_steps / steps_per_unit_m2;

    String jsonString;
    serializeJson(doc, jsonString);
    ws.textAll(jsonString); // Envía el JSON a todos los clientes conectados
}

// Manejador de eventos del WebSocket
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
    if(type == WS_EVT_CONNECT){
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        // Envía las posiciones iniciales al cliente recién conectado
        sendMotorPositions();
    } else if(type == WS_EVT_DISCONNECT){
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    } else if(type == WS_EVT_DATA){
        // Aquí puedes manejar los datos recibidos del cliente si fuera necesario
        // Por ahora, solo enviamos datos del servidor al cliente
    } else if(type == WS_EVT_PONG){
        // Pong recibido
    } else if(type == WS_EVT_ERROR){
        Serial.printf("WebSocket client #%u error(%u): %s\n", client->id(), *((uint16_t*)arg), (char*)data);
    }
}

// Variables para botones físicos (debounce)
unsigned long lastStartPress = 0;
unsigned long lastStopPress = 0;
unsigned long lastEmergencyPress = 0;
const unsigned long debounceDelay = 200;
bool lastStartState = HIGH;
bool lastStopState = HIGH;
bool lastEmergencyState = HIGH;

void setup() {
    Serial.begin(115200);

    // 1. Iniciar Sistema de Archivos y Preferencias
    if (!LittleFS.begin(true)) {
        Serial.println("Error al montar LittleFS");
    }
    pref.begin("cnc-data", false);

    // 2. Configuración de Pines (Definidos en config.h)
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(STEP2_PIN, OUTPUT);
    pinMode(DIR2_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH2_PIN, INPUT_PULLUP);
    pinMode(ENABLE2_PIN, OUTPUT);
    digitalWrite(ENABLE2_PIN, HIGH);
    pinMode(SLEEP2_PIN, OUTPUT);
    // Ensure ENABLE_PIN for Motor 1 is also initialized and set to HIGH (disabled)
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);
    digitalWrite(SLEEP2_PIN, LOW);
    
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

    // 3. Cargar valores guardados de Preferences
    currentPosition_steps = pref.getLong("pos1", 0);
    motor_speed_us = pref.getInt("speed1", 400);
    autoDistance_inches = pref.getFloat("dist1", 0.4);
    delay_at_end_ms = pref.getULong("delayEnd1", 0);
    delay_at_start_ms = pref.getULong("delayStart1", 0);
    
    currentPosition2_steps = pref.getLong("pos2", 0);
    motor2_speed_us = pref.getInt("speed2", 800);
    autoDistance2_inches = pref.getFloat("dist2", 0.4);
    delay_at_end2_ms = pref.getULong("delayEnd2", 0);
    delay_at_start2_ms = pref.getULong("delayStart2", 0);

    // 4. Configurar Timers de Hardware (Sintaxis compatible con v2.0.14)
    // Motor 1
    stepTimer = timerBegin(0, 80, true); 
    timerAttachInterrupt(stepTimer, &onStepTimer, true);
    timerAlarmWrite(stepTimer, motor_speed_us, true);
    timerAlarmEnable(stepTimer);

    // Motor 2
    stepTimer2 = timerBegin(1, 80, true);
    timerAttachInterrupt(stepTimer2, &onStepTimer2, true);
    timerAlarmWrite(stepTimer2, motor2_speed_us, true);
    timerAlarmEnable(stepTimer2);

    // Inicialmente apagados hasta recibir orden
    timerAlarmDisable(stepTimer);
    timerAlarmDisable(stepTimer2);

    // 5. Conectar WiFi
    WiFi.begin(WIFI_SSID, WIFI_PW);
    Serial.print("Conectando a WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConectado. IP: " + WiFi.localIP().toString());

    // 5.5 Configurar OTA (Over-The-Air Updates)
    ArduinoOTA.setHostname("Corte-Bobinas-OTA");
    ArduinoOTA.onStart([]() {
        Serial.println("\n\n=== Actualizando por OTA ===");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("=== OTA Completado - Reiniciando ===\n\n");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error OTA [%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("OTA activado - Actualizable por WiFi");
    
    // 6. Configurar WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws); // Añade el manejador WebSocket al servidor

    server.on("/getsettings", HTTP_GET, [](AsyncWebServerRequest *request){
        JsonDocument doc;
        doc["dist1"] = autoDistance_inches;
        doc["speed1"] = motor_speed_us;
        doc["delayEnd1"] = delay_at_end_ms;
        doc["delayStart1"] = delay_at_start_ms;
        doc["dist2"] = autoDistance2_inches;
        doc["speed2"] = motor2_speed_us;
        doc["delayEnd2"] = delay_at_end2_ms;
        doc["delayStart2"] = delay_at_start2_ms;
        String output;
        serializeJson(doc, output);
        request->send(200, "application/json", output);
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if (LittleFS.exists("/index.html")) {
        request->send(LittleFS, "/index.html", "text/html");
    } 
    else if (LittleFS.exists("index.html")) {
        request->send(LittleFS, "index.html", "text/html");
    }
    else {
        // Si llega aquí, es que LittleFS está realmente vacío
        String errorMsg = "Error: index.html no existe en la memoria.\n";
        errorMsg += "Archivos encontrados:\n";
        
        File root = LittleFS.open("/");
        File file = root.openNextFile();
        while(file){
            errorMsg += String(file.name()) + " (" + String(file.size()) + " bytes)\n";
            file = root.openNextFile();
        }
        request->send(404, "text/plain", errorMsg);
    }
});
    server.on("/startboth", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("dist1")) autoDistance_inches = request->arg("dist1").toFloat();
        if(request->hasArg("dist2")) autoDistance2_inches = request->arg("dist2").toFloat();
        startBothMotors();
        request->send(200, "text/plain", "OK");
    });

    server.on("/stopboth", HTTP_GET, [](AsyncWebServerRequest *request){
        stopBothMotors();
        request->send(200, "text/plain", "OK");
        digitalWrite(ENABLE_PIN, LOW); // Ensure motor 1 is enabled for returning home
        digitalWrite(ENABLE2_PIN, LOW); // Ensure motor 2 is enabled for returning home
    });

    server.on("/stopall", HTTP_GET, [](AsyncWebServerRequest *request){
        // Paro de emergencia: emergencyStop se activa PRIMERO para detener ISRs inmediatamente
        emergencyStop = true;
        updateStepTimer(false);
        updateStepTimer2(false);
        currentState  = IDLE;
        currentState2 = IDLE2;
        digitalWrite(ENABLE_PIN,  HIGH);
        digitalWrite(ENABLE2_PIN, HIGH);
        request->send(200, "text/plain", "OK");
    });

    server.on("/saveMem", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL, 
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            int num = request->arg("num").toInt();
            String jsonBody = "";
            for (size_t i = 0; i < len; i++) jsonBody += (char)data[i];
            String key = "mem" + String(num);
            pref.putString(key.c_str(), jsonBody);
            request->send(200, "text/plain", "OK");
    });

    server.on("/loadMem", HTTP_GET, [](AsyncWebServerRequest *request){
        int num = request->arg("num").toInt();
        String key = "mem" + String(num);
        String json = pref.getString(key.c_str(), "{}");
        request->send(200, "application/json", json);
    });

    // --- ENDPOINTS PARA HOMING ---
    server.on("/homing", HTTP_GET, [](AsyncWebServerRequest *request){
        emergencyStop = false;
        currentPosition_steps = 0;
        currentState = HOMING;
        updateStepTimer(true);
        digitalWrite(ENABLE_PIN, LOW); // Enable motor 1
        request->send(200, "text/plain", "Homing Motor 1 preciso iniciado");
    });

    server.on("/homingmotor2", HTTP_GET, [](AsyncWebServerRequest *request){
        // Inicia homing preciso del motor 2
        homingMotor2();
        request->send(200, "text/plain", "Homing Motor 2 preciso iniciado");
    });

    // --- ENDPOINTS PARA VELOCIDAD ---
    server.on("/speed1", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("val")) {
            int speed = request->arg("val").toInt();
            if(speed >= 50) {
                motor_speed_us = speed;
                pref.putInt("speed1", speed);
                if (currentState != IDLE) timerAlarmWrite(stepTimer, motor_speed_us, true);
            }
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/speed2", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("val")) {
            int speed = request->arg("val").toInt();
            if(speed >= 50) {
                motor2_speed_us = speed;
                pref.putInt("speed2", speed);
                if (currentState2 != IDLE2) timerAlarmWrite(stepTimer2, motor2_speed_us, true);
            }
        }
        request->send(200, "text/plain", "OK");
    });

    // --- ENDPOINTS PARA DISTANCIA ---
    server.on("/setdist1", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("val")) {
            autoDistance_inches = request->arg("val").toFloat();
            pref.putFloat("dist1", autoDistance_inches);
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/setdist2", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("val")) {
            autoDistance2_inches = request->arg("val").toFloat();
            pref.putFloat("dist2", autoDistance2_inches);
        }
        request->send(200, "text/plain", "OK");
    });

    // --- ENDPOINTS PARA PAUSAS ---
    server.on("/delaystart1", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("val")) {
            delay_at_start_ms = request->arg("val").toInt();
            pref.putULong("delayStart1", delay_at_start_ms);
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/delayend1", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("val")) {
            delay_at_end_ms = request->arg("val").toInt();
            pref.putULong("delayEnd1", delay_at_end_ms);
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/delaystart2", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("val")) {
            delay_at_start2_ms = request->arg("val").toInt();
            pref.putULong("delayStart2", delay_at_start2_ms);
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/delayend2", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("val")) {
            delay_at_end2_ms = request->arg("val").toInt();
            pref.putULong("delayEnd2", delay_at_end2_ms);
        }
        request->send(200, "text/plain", "OK");
    });

    // --- ENDPOINTS PARA CONTROL MANUAL (ADELANTE/ATRÁS) ---
    server.on("/moveby1", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("dir") && request->hasArg("dist")) {
            String dir = request->arg("dir");
            float dist = request->arg("dist").toFloat();
            long steps = (long)(dist * steps_per_unit_m1);
            
            goingForward = (dir == "forward");
            motor1_direction_multiplier = goingForward ? 1 : -1; // Set direction for ISR
            if (goingForward) {
                currentPosition_steps = 0;
                targetPosition_steps = steps;
            } else {
                // Para ir hacia atrás, invertir la lógica
                currentPosition_steps = steps;
                targetPosition_steps = 0;
            }
            currentState = MANUAL_MOVE;
            updateStepTimer(true);
            digitalWrite(ENABLE_PIN, LOW); // Enable motor 1
        }
        request->send(200, "text/plain", "OK");
    });

    server.on("/moveby2", HTTP_GET, [](AsyncWebServerRequest *request){
        if(request->hasArg("dir") && request->hasArg("dist")) {
            String dir = request->arg("dir");
            float dist = request->arg("dist").toFloat();
            long steps = (long)(dist * steps_per_unit_m2);
            
            goingForward2 = (dir == "forward");
            motor2_direction_multiplier = goingForward2 ? 1 : -1; // Set direction for ISR
            if (goingForward2) {
                currentPosition2_steps = 0;
                targetPosition2_steps = steps;
            } else {
                // Para ir hacia atrás, invertir la lógica
                currentPosition2_steps = steps;
                targetPosition2_steps = 0;
            }
            currentState2 = MANUAL_MOVE2;
            updateStepTimer2(true);
            digitalWrite(ENABLE2_PIN, LOW); // Enable motor 2
        }
        request->send(200, "text/plain", "OK");
    });

    // --- ENDPOINTS PARA MOVIMIENTO CONTINUO (PRESIÓN LARGA) ---
    server.on("/movefast1", HTTP_GET, [](AsyncWebServerRequest *request){
        emergencyStop = false;
        if(request->hasArg("dir")) {
            String dir = request->arg("dir");
            goingForward = (dir == "forward");
            motor1_direction_multiplier = goingForward ? 1 : -1; // Set direction for ISR
            currentState = MANUAL_FAST_MOVE;
            updateStepTimer(true);
            digitalWrite(ENABLE_PIN, LOW); // Enable motor 1
        }
        request->send(200, "text/plain", "OK");
    });
    
    server.on("/stopfast1", HTTP_GET, [](AsyncWebServerRequest *request){
        updateStepTimer(false);
        currentState = IDLE;
        request->send(200, "text/plain", "OK");
    });

    server.on("/movefast2", HTTP_GET, [](AsyncWebServerRequest *request){
        emergencyStop = false;
        if(request->hasArg("dir")) {
            String dir = request->arg("dir");
            goingForward2 = (dir == "forward");
            motor2_direction_multiplier = goingForward2 ? 1 : -1; // Set direction for ISR
            currentState2 = MANUAL_FAST_MOVE2;
            updateStepTimer2(true);
            digitalWrite(ENABLE2_PIN, LOW); // Enable motor 2
        }
        request->send(200, "text/plain", "OK");
    });
    
    server.on("/stopfast2", HTTP_GET, [](AsyncWebServerRequest *request){
        updateStepTimer2(false);
        currentState2 = IDLE2;
        request->send(200, "text/plain", "OK");
    });

    // --- ENDPOINTS PARA CICLOS AUTOMÁTICOS (POR MOTOR) ---
    server.on("/startauto1", HTTP_GET, [](AsyncWebServerRequest *request){
        emergencyStop = false;
        if(request->hasArg("dist")) {
            autoDistance_inches = request->arg("dist").toFloat();
            pref.putFloat("dist1", autoDistance_inches);
        }
        currentPosition_steps = 0;
        targetPosition_steps = (long)(autoDistance_inches * steps_per_unit_m1);
        goingForward = true;
        currentState = RUNNING;
        motor1_direction_multiplier = 1; // Set direction for ISR
        updateStepTimer(true);
        request->send(200, "text/plain", "OK");
    });

    server.on("/stopauto1", HTTP_GET, [](AsyncWebServerRequest *request){
        targetPosition_steps = 0;
        currentState = RETURNING_HOME;
        motor1_direction_multiplier = -1; // Set direction for ISR (returning home is backward)
        updateStepTimer(true);
        request->send(200, "text/plain", "OK");
    });

    server.on("/startauto2", HTTP_GET, [](AsyncWebServerRequest *request){
        emergencyStop = false;
        if(request->hasArg("dist")) {
            autoDistance2_inches = request->arg("dist").toFloat();
            pref.putFloat("dist2", autoDistance2_inches);
        }
        currentPosition2_steps = 0;
        targetPosition2_steps = (long)(autoDistance2_inches * steps_per_unit_m2);
        goingForward2 = true;
        currentState2 = RUNNING2;
        motor2_direction_multiplier = 1; // Set direction for ISR
        updateStepTimer2(true);
        request->send(200, "text/plain", "OK");
    });

    server.on("/stopauto2", HTTP_GET, [](AsyncWebServerRequest *request){
        targetPosition2_steps = 0;
        currentState2 = RETURNING_HOME2;
        motor2_direction_multiplier = -1; // Set direction for ISR (returning home is backward)
        updateStepTimer2(true);
        request->send(200, "text/plain", "OK");
    });

    server.begin();
}

void loop() {
    // Manejar OTA updates
    ArduinoOTA.handle();

    // Limpiar clientes WebSocket desconectados
    ws.cleanupClients();
    // Enviar posiciones de motores periódicamente a través de WebSocket
    static unsigned long lastWsSendTime = 0;
    if (millis() - lastWsSendTime > 100) { // Envía actualizaciones cada 100ms
        sendMotorPositions();
        lastWsSendTime = millis();
    }
    
    handleMotor();
    handleMotor2();
    checkSync();

    // Lógica de botones físicos
    bool startRead = digitalRead(START_BUTTON_PIN);
    bool stopRead = digitalRead(STOP_BUTTON_PIN);
    unsigned long now = millis();

    if (startRead == LOW && lastStartState == HIGH && (now - lastStartPress > debounceDelay)) {
        lastStartPress = now;
        startBothMotors();
    }
    lastStartState = startRead;

    if (stopRead == LOW && lastStopState == HIGH && (now - lastStopPress > debounceDelay)) {
        lastStopPress = now;
        stopBothMotors();
    }
    lastStopState = stopRead;

    bool emergencyRead = digitalRead(EMERGENCY_STOP_PIN);
    if (emergencyRead == LOW && lastEmergencyState == HIGH && (now - lastEmergencyPress > debounceDelay)) {
        lastEmergencyPress = now;
        emergencyStop = true;   // detener ISRs inmediatamente
        updateStepTimer(false);
        updateStepTimer2(false);
        currentState  = IDLE;
        currentState2 = IDLE2;
        digitalWrite(ENABLE_PIN,  HIGH);
        digitalWrite(ENABLE2_PIN, HIGH);
    }
    lastEmergencyState = emergencyRead;

    // Guardado de posición en Preferences al entrar en IDLE
    static State lastS = IDLE;
    if (currentState == IDLE && lastS != IDLE) pref.putLong("pos1", currentPosition_steps);
    lastS = currentState;
    static State2 lastS2 = IDLE2;
    if (currentState2 == IDLE2 && lastS2 != IDLE2) pref.putLong("pos2", currentPosition2_steps);
    lastS2 = currentState2;
}