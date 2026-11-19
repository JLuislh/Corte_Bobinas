#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <AsyncWebSocket.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "config.h"
#include "motor_logic.h"

// Instancias Globales
AsyncWebServer server(80);
Preferences pref;
AsyncWebSocket ws("/ws");

// --- COLA DE COMANDOS ---
// Los handlers HTTP corren en la tarea de AsyncTCP; para no modificar el estado de
// los motores en paralelo con loop(), encolan el comando y loop() lo ejecuta.
enum CmdType : uint8_t {
    CMD_START_BOTH, CMD_STOP_BOTH,
    CMD_HOME1, CMD_HOME2,
    CMD_SPEED1, CMD_SPEED2,
    CMD_MOVEBY1, CMD_MOVEBY2,
    CMD_FAST1, CMD_FAST2, CMD_STOPFAST1, CMD_STOPFAST2,
    CMD_STARTAUTO1, CMD_STARTAUTO2, CMD_STOPAUTO1, CMD_STOPAUTO2
};

struct Command {
    CmdType type;
    float a;   // distancia (NAN = no indicada)
    float b;   // segunda distancia (NAN = no indicada)
    int i;     // velocidad / dirección (1 = adelante)
};

static QueueHandle_t cmdQueue = nullptr;

// El paro de emergencia no pasa por la cola: el flag detiene las ISRs al instante
// y loop() completa el paro después de procesar la cola, así siempre gana.
static volatile bool estopRequested = false;

static void requestEmergencyStop() {
    emergencyStop = true;
    estopRequested = true;
}

static void enqueue(AsyncWebServerRequest *request, CmdType type, float a = NAN, float b = NAN, int i = 0) {
    Command c{type, a, b, i};
    if (xQueueSend(cmdQueue, &c, 0) == pdTRUE) {
        request->send(200, "text/plain", "OK");
    } else {
        request->send(503, "text/plain", "BUSY");
    }
}

static float optFloatArg(AsyncWebServerRequest *request, const char *name) {
    return request->hasArg(name) ? request->arg(name).toFloat() : NAN;
}

static void processCommands() {
    Command c;
    while (xQueueReceive(cmdQueue, &c, 0) == pdTRUE) {
        switch (c.type) {
        case CMD_START_BOTH:
            // Solo aplica distancias si realmente arranca (START en marcha no hace nada)
            if (currentState == IDLE && currentState2 == IDLE2) {
                if (!isnan(c.a)) autoDistance_inches = c.a;
                if (!isnan(c.b)) autoDistance2_inches = c.b;
                startBothMotors();
            }
            break;
        case CMD_STOP_BOTH:  stopBothMotors(); break;
        case CMD_HOME1:      homingMotor1(); break;
        case CMD_HOME2:      homingMotor2(); break;
        case CMD_SPEED1:
            if (c.i >= 50) { setSpeed1(c.i); pref.putInt("speed1", c.i); }
            break;
        case CMD_SPEED2:
            if (c.i >= 50) { setSpeed2(c.i); pref.putInt("speed2", c.i); }
            break;
        case CMD_MOVEBY1:    moveBy1(c.i == 1, c.a); break;
        case CMD_MOVEBY2:    moveBy2(c.i == 1, c.a); break;
        case CMD_FAST1:      fastMove1(c.i == 1); break;
        case CMD_FAST2:      fastMove2(c.i == 1); break;
        case CMD_STOPFAST1:  stopFast1(); break;
        case CMD_STOPFAST2:  stopFast2(); break;
        case CMD_STARTAUTO1:
            if (currentState == IDLE) {
                if (!isnan(c.a)) { autoDistance_inches = c.a; pref.putFloat("dist1", c.a); }
                startAuto1();
            }
            break;
        case CMD_STARTAUTO2:
            if (currentState2 == IDLE2) {
                if (!isnan(c.a)) { autoDistance2_inches = c.a; pref.putFloat("dist2", c.a); }
                startAuto2();
            }
            break;
        case CMD_STOPAUTO1:  stopAuto1(); break;
        case CMD_STOPAUTO2:  stopAuto2(); break;
        }
    }
}

// Envía las posiciones de los motores a todos los clientes WebSocket
void sendMotorPositions() {
    if (ws.count() == 0) return;

    JsonDocument doc;
    doc["pos1"] = (float)currentPosition_steps / steps_per_unit_m1;
    doc["pos2"] = (float)currentPosition2_steps / steps_per_unit_m2;

    String jsonString;
    serializeJson(doc, jsonString);
    ws.textAll(jsonString);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    } else if (type == WS_EVT_ERROR) {
        Serial.printf("WebSocket client #%u error(%u): %s\n", client->id(), *((uint16_t *)arg), (char *)data);
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

static bool otaStarted = false;

void setup() {
    Serial.begin(115200);

    cmdQueue = xQueueCreate(16, sizeof(Command));

    // 1. Sistema de archivos y preferencias
    if (!LittleFS.begin(true)) {
        Serial.println("Error al montar LittleFS");
    }
    pref.begin("cnc-data", false);

    // 2. Pines (motores deshabilitados al arrancar)
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(STEP2_PIN, OUTPUT);
    pinMode(DIR2_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH2_PIN, INPUT_PULLUP);
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);
    pinMode(ENABLE2_PIN, OUTPUT);
    digitalWrite(ENABLE2_PIN, HIGH);
    pinMode(SLEEP2_PIN, OUTPUT);
    digitalWrite(SLEEP2_PIN, LOW);

    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

    // 3. Valores guardados
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

    // 4. Timers de hardware (API arduino-esp32 v2.x), apagados hasta recibir orden
    stepTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(stepTimer, &onStepTimer, true);
    timerAlarmWrite(stepTimer, motor_speed_us, true);
    timerAlarmDisable(stepTimer);

    stepTimer2 = timerBegin(1, 80, true);
    timerAttachInterrupt(stepTimer2, &onStepTimer2, true);
    timerAlarmWrite(stepTimer2, motor2_speed_us, true);
    timerAlarmDisable(stepTimer2);

    // 5. WiFi sin bloquear: los botones físicos y el paro de emergencia funcionan
    //    aunque no haya red. La conexión y el OTA se completan desde loop().
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PW);
    Serial.println("Conectando a WiFi (en segundo plano)...");

    // 5.5 OTA (se inicia en loop() cuando hay conexión)
    ArduinoOTA.setHostname("Corte-Bobinas-OTA");
    ArduinoOTA.onStart([]() {
        emergencyStopAll();  // nunca actualizar con motores en movimiento
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

    // 6. WebSocket y endpoints
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/getsettings", HTTP_GET, [](AsyncWebServerRequest *request) {
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

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (LittleFS.exists("/index.html")) {
            request->send(LittleFS, "/index.html", "text/html");
        } else {
            String errorMsg = "Error: index.html no existe en la memoria.\nArchivos encontrados:\n";
            File root = LittleFS.open("/");
            File file = root.openNextFile();
            while (file) {
                errorMsg += String(file.name()) + " (" + String(file.size()) + " bytes)\n";
                file = root.openNextFile();
            }
            request->send(404, "text/plain", errorMsg);
        }
    });

    // --- CICLO DE AMBOS MOTORES ---
    server.on("/startboth", HTTP_GET, [](AsyncWebServerRequest *request) {
        enqueue(request, CMD_START_BOTH, optFloatArg(request, "dist1"), optFloatArg(request, "dist2"));
    });
    server.on("/stopboth", HTTP_GET, [](AsyncWebServerRequest *request) {
        enqueue(request, CMD_STOP_BOTH);
    });
    server.on("/stopall", HTTP_GET, [](AsyncWebServerRequest *request) {
        requestEmergencyStop();
        request->send(200, "text/plain", "OK");
    });

    // --- MEMORIAS ---
    server.on("/saveMem", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            // El cuerpo puede llegar en varios fragmentos: acumular en un búfer.
            // La librería libera _tempObject con free(), por eso se usa malloc.
            if (total > 2048) {
                if (index == 0) request->send(413, "text/plain", "demasiado grande");
                return;
            }
            if (index == 0) {
                request->_tempObject = malloc(total + 1);
                if (!request->_tempObject) { request->send(500, "text/plain", "sin memoria"); return; }
            }
            char *body = (char *)request->_tempObject;
            if (!body) return;
            memcpy(body + index, data, len);
            if (index + len < total) return;
            body[total] = '\0';
            int num = request->arg("num").toInt();
            if (num >= 1 && num <= 3) {
                String key = "mem" + String(num);
                pref.putString(key.c_str(), body);
                request->send(200, "text/plain", "OK");
            } else {
                request->send(400, "text/plain", "num invalido");
            }
        });

    server.on("/loadMem", HTTP_GET, [](AsyncWebServerRequest *request) {
        int num = request->arg("num").toInt();
        String key = "mem" + String(num);
        String json = pref.getString(key.c_str(), "{}");
        request->send(200, "application/json", json);
    });

    // --- HOMING ---
    server.on("/homing", HTTP_GET, [](AsyncWebServerRequest *request) {
        enqueue(request, CMD_HOME1);
    });
    server.on("/homingmotor2", HTTP_GET, [](AsyncWebServerRequest *request) {
        enqueue(request, CMD_HOME2);
    });

    // --- VELOCIDAD ---
    server.on("/speed1", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasArg("val")) { request->send(400, "text/plain", "falta val"); return; }
        enqueue(request, CMD_SPEED1, NAN, NAN, request->arg("val").toInt());
    });
    server.on("/speed2", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasArg("val")) { request->send(400, "text/plain", "falta val"); return; }
        enqueue(request, CMD_SPEED2, NAN, NAN, request->arg("val").toInt());
    });

    // --- DISTANCIA (variables atómicas; se aplican en el próximo inicio de carrera) ---
    server.on("/setdist1", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasArg("val")) {
            autoDistance_inches = request->arg("val").toFloat();
            pref.putFloat("dist1", autoDistance_inches);
        }
        request->send(200, "text/plain", "OK");
    });
    server.on("/setdist2", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasArg("val")) {
            autoDistance2_inches = request->arg("val").toFloat();
            pref.putFloat("dist2", autoDistance2_inches);
        }
        request->send(200, "text/plain", "OK");
    });

    // --- PAUSAS ---
    server.on("/delaystart1", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasArg("val")) {
            delay_at_start_ms = request->arg("val").toInt();
            pref.putULong("delayStart1", delay_at_start_ms);
        }
        request->send(200, "text/plain", "OK");
    });
    server.on("/delayend1", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasArg("val")) {
            delay_at_end_ms = request->arg("val").toInt();
            pref.putULong("delayEnd1", delay_at_end_ms);
        }
        request->send(200, "text/plain", "OK");
    });
    server.on("/delaystart2", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasArg("val")) {
            delay_at_start2_ms = request->arg("val").toInt();
            pref.putULong("delayStart2", delay_at_start2_ms);
        }
        request->send(200, "text/plain", "OK");
    });
    server.on("/delayend2", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasArg("val")) {
            delay_at_end2_ms = request->arg("val").toInt();
            pref.putULong("delayEnd2", delay_at_end2_ms);
        }
        request->send(200, "text/plain", "OK");
    });

    // --- MOVIMIENTO RELATIVO (solo con el motor en IDLE) ---
    server.on("/moveby1", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasArg("dir") || !request->hasArg("dist")) { request->send(400, "text/plain", "faltan dir/dist"); return; }
        enqueue(request, CMD_MOVEBY1, request->arg("dist").toFloat(), NAN, request->arg("dir") == "forward" ? 1 : 0);
    });
    server.on("/moveby2", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasArg("dir") || !request->hasArg("dist")) { request->send(400, "text/plain", "faltan dir/dist"); return; }
        enqueue(request, CMD_MOVEBY2, request->arg("dist").toFloat(), NAN, request->arg("dir") == "forward" ? 1 : 0);
    });

    // --- MOVIMIENTO CONTINUO (mantener presionado; requiere keep-alive) ---
    server.on("/movefast1", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasArg("dir")) { request->send(400, "text/plain", "falta dir"); return; }
        enqueue(request, CMD_FAST1, NAN, NAN, request->arg("dir") == "forward" ? 1 : 0);
    });
    server.on("/movefast2", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!request->hasArg("dir")) { request->send(400, "text/plain", "falta dir"); return; }
        enqueue(request, CMD_FAST2, NAN, NAN, request->arg("dir") == "forward" ? 1 : 0);
    });
    server.on("/keepfast1", HTTP_GET, [](AsyncWebServerRequest *request) {
        keepFast1();
        request->send(200, "text/plain", "OK");
    });
    server.on("/keepfast2", HTTP_GET, [](AsyncWebServerRequest *request) {
        keepFast2();
        request->send(200, "text/plain", "OK");
    });
    server.on("/stopfast1", HTTP_GET, [](AsyncWebServerRequest *request) {
        enqueue(request, CMD_STOPFAST1);
    });
    server.on("/stopfast2", HTTP_GET, [](AsyncWebServerRequest *request) {
        enqueue(request, CMD_STOPFAST2);
    });

    // --- CICLO AUTOMÁTICO POR MOTOR ---
    server.on("/startauto1", HTTP_GET, [](AsyncWebServerRequest *request) {
        enqueue(request, CMD_STARTAUTO1, optFloatArg(request, "dist"));
    });
    server.on("/stopauto1", HTTP_GET, [](AsyncWebServerRequest *request) {
        enqueue(request, CMD_STOPAUTO1);
    });
    server.on("/startauto2", HTTP_GET, [](AsyncWebServerRequest *request) {
        enqueue(request, CMD_STARTAUTO2, optFloatArg(request, "dist"));
    });
    server.on("/stopauto2", HTTP_GET, [](AsyncWebServerRequest *request) {
        enqueue(request, CMD_STOPAUTO2);
    });

    server.begin();
}

void loop() {
    unsigned long now = millis();

    // --- Red / OTA ---
    if (!otaStarted && WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi conectado. IP: " + WiFi.localIP().toString());
        ArduinoOTA.begin();
        otaStarted = true;
        Serial.println("OTA activado - Actualizable por WiFi");
    }
    if (otaStarted) ArduinoOTA.handle();

    ws.cleanupClients();
    static unsigned long lastWsSendTime = 0;
    if (now - lastWsSendTime > 100) {
        sendMotorPositions();
        lastWsSendTime = now;
    }

    // --- Botones físicos ---
    bool emergencyRead = digitalRead(EMERGENCY_STOP_PIN);
    if (emergencyRead == LOW && lastEmergencyState == HIGH && (now - lastEmergencyPress > debounceDelay)) {
        lastEmergencyPress = now;
        requestEmergencyStop();
    }
    lastEmergencyState = emergencyRead;

    bool startRead = digitalRead(START_BUTTON_PIN);
    if (startRead == LOW && lastStartState == HIGH && (now - lastStartPress > debounceDelay)) {
        lastStartPress = now;
        startBothMotors();  // ignorado si algún motor está en movimiento
    }
    lastStartState = startRead;

    bool stopRead = digitalRead(STOP_BUTTON_PIN);
    if (stopRead == LOW && lastStopState == HIGH && (now - lastStopPress > debounceDelay)) {
        lastStopPress = now;
        stopBothMotors();
    }
    lastStopState = stopRead;

    // --- Comandos web; el paro de emergencia se aplica al final para que siempre gane ---
    processCommands();
    if (estopRequested) {
        estopRequested = false;
        emergencyStopAll();
    }

    handleMotor();
    handleMotor2();
    checkSync();

    // Guardado de posición al entrar en IDLE
    static State lastS = IDLE;
    if (currentState == IDLE && lastS != IDLE) pref.putLong("pos1", currentPosition_steps);
    lastS = currentState;
    static State2 lastS2 = IDLE2;
    if (currentState2 == IDLE2 && lastS2 != IDLE2) pref.putLong("pos2", currentPosition2_steps);
    lastS2 = currentState2;
}
