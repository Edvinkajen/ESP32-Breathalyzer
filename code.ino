#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_MAX1704X.h>
#include <freertos/semphr.h>

const char* WIFI_SSID = "ESP32S3";
const char* WIFI_PASS = "12345678";

const int PIN_RED   = A0; 
const int PIN_GREEN = A1;
const int PIN_BLUE  = A2;
const int PIN_VIB   = A3;
const int PIN_BUTTON = A4; 
const int PIN_ALCOHOL = 5;
const int PIN_BUZZER  = 6;
const int PIN_TEMP  = 9;
const int PIN_MIC   = 10;
const int PIN_SDA   = SDA;
const int PIN_SCL   = SCL;

// Justerbara Konstanter
const float VREF_S3 = 3.3;      
const int ADC_MAX_S3 = 4095;
const float MV_PER_PPM = 0.15;   
const int VOFFSET = 1521;        

const unsigned long INACTIVITY_TIMEOUT_MS = 180000; 
const unsigned long BUTTON_HOLD_TIME_MS = 3000;    
const int MIC_THRESHOLD = 1500;
const int BLOW_TIME_REQUIRED_MS = 6000;
const int BLOW_INTERRUPTION_TOLERANCE_MS = 400;

// Filer
const char* FILE_USERS      = "/users.json";
const char* FILE_MEASUREMENTS= "/measurements.json";
const char* FILE_ACTIVE_USER = "/active.txt";

Adafruit_MAX17048 battery;
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
SemaphoreHandle_t jsonMutex;

struct AppState {
  String usersJson = "[]";
  String measurementsJson = "[]";
  String activeUserId = "";
  unsigned long lastActivityTime = 0;
  unsigned long lastBattSend = 0;
} appState;


extern bool isStabilizing; 

int64_t systemTimeOffset = 0; 
uint64_t getEpochTime() {
  if (systemTimeOffset == 0) return millis(); 
  return (uint64_t)(systemTimeOffset + millis());
}

void updateActivity() {
  appState.lastActivityTime = millis();
}

void drawText(const String& text) {
  display.clearBuffer();
  display.setFont(u8g2_font_logisoso28_tf);
  int width = display.getStrWidth(text.c_str());
  int x = (128 - width) / 2;
  display.drawStr(x, 50, text.c_str());
  display.sendBuffer();
}

void setRGBColor(int r, int g, int b) {
  ledcWrite(PIN_RED, 255 - r);
  ledcWrite(PIN_GREEN, 255 - g);
  ledcWrite(PIN_BLUE, 255 - b);
}

void setupRGB() {
  if (!ledcAttach(PIN_RED, 5000, 8)) Serial.println("Failed to attach RED pin");
  if (!ledcAttach(PIN_GREEN, 5000, 8)) Serial.println("Failed to attach GREEN pin");
  if (!ledcAttach(PIN_BLUE, 5000, 8)) Serial.println("Failed to attach BLUE pin");
  setRGBColor(0, 0, 0);
}

void setRGBFromHex(String hexColor) {
  if (hexColor.startsWith("#")) {
    hexColor = hexColor.substring(1);
  }
  if (hexColor.length() < 6) {
    setRGBColor(0, 0, 0);
    return;
  }
  long number = strtol(hexColor.c_str(), NULL, 16);
  int r = (number >> 16) & 0xFF;
  int g = (number >> 8) & 0xFF;
  int b = number & 0xFF;
  setRGBColor(r, g, b);
}

void updateLEDForActiveUser() {
  if (xSemaphoreTake(jsonMutex, portMAX_DELAY)) {
    String activeId = appState.activeUserId;
    String usersStr = appState.usersJson;
    xSemaphoreGive(jsonMutex);

    if (activeId == "") {
      setRGBColor(0, 0, 0);
      return;
    }

    DynamicJsonDocument users(8192);
    deserializeJson(users, usersStr);
    JsonArray arr = users.as<JsonArray>();

    bool found = false;
    for (JsonObject user : arr) {
      String id = user["id"] | "";
      if (id == activeId) {
        String color = user["color"] | "#000000";
        setRGBFromHex(color);
        found = true;
        break;
      }
    }
    if (!found) setRGBColor(0, 0, 0);
  }
}

String readFile(const char* path, const String& defVal = "[]") {
  if (!LittleFS.exists(path)) return defVal;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return defVal;
  String out = f.readString();
  f.close();
  out.trim();
  return (out == "") ? defVal : out;
}

void writeFile(const char* path, const String& data) {
  File f = LittleFS.open(path, FILE_WRITE);
  if (f) {
    f.print(data);
    f.close();
  }
}

void loadAllData() {
  if (xSemaphoreTake(jsonMutex, portMAX_DELAY)) {
    appState.usersJson = readFile(FILE_USERS, "[]");
    appState.measurementsJson = readFile(FILE_MEASUREMENTS, "[]");
    
    if (LittleFS.exists(FILE_ACTIVE_USER)) {
      File f = LittleFS.open(FILE_ACTIVE_USER, FILE_READ);
      appState.activeUserId = f.readString();
      appState.activeUserId.trim();
      f.close();
    } else {
      appState.activeUserId = "";
    }
    xSemaphoreGive(jsonMutex);
  }
  updateLEDForActiveUser();
}

void broadcastState() {
  DynamicJsonDocument doc(32768); 
  DynamicJsonDocument usersDoc(8192);
  DynamicJsonDocument measDoc(16384);

  if (xSemaphoreTake(jsonMutex, portMAX_DELAY)) {
    deserializeJson(usersDoc, appState.usersJson);
    deserializeJson(measDoc, appState.measurementsJson);
    
    doc["type"] = "state";
    doc["users"] = usersDoc.as<JsonArray>();
    doc["measurements"] = measDoc.as<JsonArray>();
    doc["activeUserId"] = appState.activeUserId;
    xSemaphoreGive(jsonMutex);

    String json;
    serializeJson(doc, json);
    ws.textAll(json);
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
              AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    broadcastState();
    return;
  }
  if (type != WS_EVT_DATA) return;

  String msg = "";
  for (size_t i = 0; i < len; i++) msg += (char)data[i];

  DynamicJsonDocument doc(2048);
  deserializeJson(doc, msg);
  String t = doc["type"] | "";

  if (xSemaphoreTake(jsonMutex, portMAX_DELAY)) {
    if (t == "add_user") {
      DynamicJsonDocument usersDoc(8192);
      deserializeJson(usersDoc, appState.usersJson);
      JsonArray arr = usersDoc.as<JsonArray>();
      arr.add(doc["user"]);
      
      String newUsers;
      serializeJson(arr, newUsers);
      appState.usersJson = newUsers;
      writeFile(FILE_USERS, newUsers);
    }
    else if (t == "set_active_user") {
      appState.activeUserId = doc["activeUserId"].as<String>();
      writeFile(FILE_ACTIVE_USER, appState.activeUserId);
    }
    else if (t == "sync_time") {
      uint64_t browserTime = doc["timestamp"].as<uint64_t>();
      systemTimeOffset = browserTime - millis();
      Serial.printf("Time synced! Offset: %lld\n", systemTimeOffset);
    }
    else if (t == "add_measurement") { 
      DynamicJsonDocument measDoc(16384);
      deserializeJson(measDoc, appState.measurementsJson);
      JsonArray arr = measDoc.as<JsonArray>();
      JsonObject m = arr.createNestedObject();
      
      m["timestamp"] = getEpochTime();
      m["userId"]   = appState.activeUserId;
      m["value"]    = doc["value"];

      String newMeas;
      serializeJson(arr, newMeas);
      appState.measurementsJson = newMeas;
      writeFile(FILE_MEASUREMENTS, newMeas);
    }
    else if (t == "clear_all") {
      appState.usersJson = "[]";
      appState.measurementsJson = "[]";
      appState.activeUserId = "";
      writeFile(FILE_USERS, "[]");
      writeFile(FILE_MEASUREMENTS, "[]");
      writeFile(FILE_ACTIVE_USER, "");
    }
    xSemaphoreGive(jsonMutex);
  }

  if (t == "set_active_user" || t == "clear_all") {
    updateLEDForActiveUser();
  }
  broadcastState();
}

void sendBatteryWS() {
  float pct = battery.cellPercent();
  StaticJsonDocument<100> doc;
  doc["type"] = "battery";
  doc["percent"] = pct;
  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

void sendMeasurementToWeb(float promilleValue) {
  if (xSemaphoreTake(jsonMutex, portMAX_DELAY)) {
    if (appState.activeUserId == "") {
      Serial.println("No active user - cannot save measurement.");
      xSemaphoreGive(jsonMutex);
      return;
    }
    
    StaticJsonDocument<200> eventDoc;
    eventDoc["type"] = "add_measurement";
    eventDoc["value"] = promilleValue;
    eventDoc["timestamp"] = getEpochTime();
    eventDoc["userId"] = appState.activeUserId;
    
    DynamicJsonDocument measDoc(16384);
    deserializeJson(measDoc, appState.measurementsJson);
    JsonArray arr = measDoc.as<JsonArray>();
    JsonObject m = arr.createNestedObject();
    
    m["timestamp"] = getEpochTime();
    m["userId"]   = appState.activeUserId;
    m["value"]    = promilleValue;
    
    String newMeas;
    serializeJson(arr, newMeas);
    appState.measurementsJson = newMeas;
    writeFile(FILE_MEASUREMENTS, newMeas);
    
    xSemaphoreGive(jsonMutex);

    String jsonString;
    serializeJson(eventDoc, jsonString);
    ws.textAll(jsonString);
    broadcastState();
  }
}

class SensorStabilizer {
private:
  float filteredValue = 0;    
  int peakValue = 0;          
  int baseline = 0;           
  unsigned long startTime = 0;
  bool firstRun = true;       

  const int OVERSAMPLE_COUNT = 64; 
  const float FILTER_FACTOR = 0.1;   
  const unsigned long MAX_MEASURE_TIME_MS = 20000; // Max tid vi letar peak
  const int DROP_THRESHOLD_MV = 10;  

public:
  void reset() {
    peakValue = -9999; 
    firstRun = true;
    startTime = 0;
  }

  int readMilliVoltsSmooth(int pin) {
    long sum = 0;
    for(int i = 0; i < OVERSAMPLE_COUNT; i++) {
      sum += analogReadMilliVolts(pin); 
    }
    return (int)(sum / OVERSAMPLE_COUNT);
  }

  bool update(int pin, float &promilleOut) {
    int currentMv = readMilliVoltsSmooth(pin);
    
    if (firstRun) {
      filteredValue = currentMv;
      
      // (1805 / 4095) * 3300mV ≈ 1454 mV
      baseline = VOFFSET;
      peakValue = currentMv; 
      startTime = millis();
      firstRun = false;
      Serial.printf("Analysis Started. Fixed Baseline: %d mV (Current: %d mV)\n", baseline, currentMv);
    }

    filteredValue = (filteredValue * (1.0 - FILTER_FACTOR)) + (currentMv * FILTER_FACTOR);
    int smoothedMv = (int)filteredValue;

    if (smoothedMv > peakValue) {
      peakValue = smoothedMv;
    }
    
    Serial.printf("Raw:%d,Smooth:%d,Peak:%d,Base:%d\n", currentMv, smoothedMv, peakValue, baseline);
    
    unsigned long timeElapsed = millis() - startTime;
    bool isDone = false;
    
    if (timeElapsed > 1000 && (peakValue > baseline + 10) && (smoothedMv < peakValue - DROP_THRESHOLD_MV)) {
      Serial.println(">>> Peak found (Drop detected) <<<");
      isDone = true;
    }
    
    if (timeElapsed >= MAX_MEASURE_TIME_MS) {
      Serial.println(">>> Timeout reached - taking max value found <<<");
      isDone = true;
    }

    if (isDone) {
      
      float voltageDelta = (float)(peakValue - baseline);
      if (voltageDelta < 0) voltageDelta = 0;

      float ppm = voltageDelta / MV_PER_PPM; 
      float calculatedPromille = ppm / 450.0; 

      float temp = readTemperature(PIN_TEMP);
      float factor = getSensitivityFactor(temp);
      calculatedPromille = calculatedPromille / factor;
      
      promilleOut = round(calculatedPromille * 100.0) / 100.0;
      if (promilleOut < 0) promilleOut = 0;

      Serial.printf("RESULT: Peak: %d mV, Baseline: %d mV, Delta: %.1f mV -> %.2f Promille\n", 
                    peakValue, baseline, voltageDelta, promilleOut);
      return true;
    }

    return false;
  }
};

SensorStabilizer alcoholSensor;

class VibrationMotor {
private:
  bool running = false;
  unsigned long startTime = 0;
  int stepCount = 0;
public:
  void start() {
    running = true;
    startTime = millis();
    stepCount = 0;
    digitalWrite(PIN_VIB, HIGH);
  }
  void update() {
    if (!running) return;
    if (millis() - startTime >= 100) {
      startTime = millis();
      stepCount++;
      digitalWrite(PIN_VIB, stepCount % 2 == 0 ? HIGH : LOW);
      if (stepCount >= 6) {
        running = false;
        digitalWrite(PIN_VIB, LOW);
      }
    }
  }
};

VibrationMotor vibMotor;

class SensorResetManager {
private:
  bool active = false;
  unsigned long startTime = 0;
  int baseline = 0;
  int tolerance = 0;
  unsigned long timeout = 0;
public:
  void start(int _baseline, int _tolerance = 20, unsigned long _timeout = 90000) {
    active = true;
    startTime = millis();
    baseline = _baseline;
    tolerance = _tolerance;
    timeout = _timeout;
    drawText("RESET...");
    Serial.println("Starting sensor reset...");
  }
  bool isActive() { return active; }
  
  bool update(int pin) {
    updateActivity();
    if (!active) return false;
    
    int v = analogRead(pin);
    if (abs(v - baseline) <= tolerance) {
      active = false;
      Serial.printf("Sensor reset OK! ADC=%d (baseline=%d)\n", v, baseline);
      return true;
    }
    if (millis() - startTime >= timeout) {
      active = false;
      Serial.printf("Sensor reset TIMEOUT! ADC=%d\n", v);
      return true;
    }
    return false;
  }
};

SensorResetManager sensorReset;

float readTemperature(int pin) {
  uint32_t voltage_mV = analogReadMilliVolts(pin);
  float tempC = (voltage_mV - 500.0) / 10.0;
  return tempC;
}

float getSensitivityFactor(float temp) {
  const float A = 107.5777;
  const float B = 17.37397;
  const float DIVISOR = 14.44597;
  float denominator = pow(2.0, temp / DIVISOR);
  float percent = A - (B / denominator);

  if (percent < 10.0) percent = 10.0;
  if (percent > 150.0) percent = 150.0;

  return percent / 100.0;
}

float calculatePromille(int rawValue) {
  float voltage = ((rawValue - VOFFSET) * VREF_S3) / ADC_MAX_S3;  
  float mV = voltage * 1000.0;
  float ppm = mV / MV_PER_PPM;
  float promille = ppm / 450.0; 

  float Temp = readTemperature(PIN_TEMP);
  float Factor = getSensitivityFactor(Temp);
  Serial.printf("Temperatur: %.2f",Temp);
  Serial.printf("Factor: %.2f",Factor);

  promille = promille / Factor;
  promille = round(promille * 100) / 100;
  if (promille < 0) promille = 0;
  return promille;
}

void enterDeepSleep() {
  Serial.println("Entering deep sleep...");
  setRGBColor(0, 0, 0);
  drawText("Zzz...");
  delay(1000);
  display.clear();
  
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  esp_sleep_enable_ext1_wakeup(1ULL << PIN_BUTTON, ESP_EXT1_WAKEUP_ALL_LOW);
  
  Serial.println("Deep sleep activated.");
  delay(100);
  esp_deep_sleep_start();
}

void showBattery() {
  updateActivity();
  float percent = battery.cellPercent();
  float voltage = battery.cellVoltage(); 
  
  display.clearBuffer();
  display.setFont(u8g2_font_logisoso28_tf);
  
  String text = String((int)percent) + "%";
  int w = display.getStrWidth(text.c_str());
  int x = (128 - w) / 2;
  display.drawStr(x, 40, text.c_str());
  display.sendBuffer();
  
  Serial.printf("Battery: %.2f%% (%.2f V)\n", percent, voltage);
  
  delay(2000); 
  display.clearBuffer();
  display.sendBuffer();
}

void checkDeepSleepConditions() {
  if (isStabilizing) return; 
  
  bool pressed = (digitalRead(PIN_BUTTON) == LOW);
  
  static bool initialized = false;
  static bool ignoreCurrentPress = false;
  static bool wasPressed = false;
  static unsigned long pressStart = 0;
  
  if (!initialized) {
    if (pressed) {
      ignoreCurrentPress = true;
      wasPressed = true;
    }
    initialized = true;
  }

  if (pressed) {
    if (!wasPressed) {
      pressStart = millis();
      wasPressed = true;
      ignoreCurrentPress = false; 
      updateActivity();
    }
    
    if (!ignoreCurrentPress) {
      if (millis() - pressStart >= BUTTON_HOLD_TIME_MS) {
        enterDeepSleep();
      }
    }
  } else {
    if (wasPressed) {
      if (!ignoreCurrentPress) {
        unsigned long held = millis() - pressStart;
        if (held < BUTTON_HOLD_TIME_MS) {
          showBattery();
        }
      }
      wasPressed = false;
      ignoreCurrentPress = false; 
    }
  }
  
  if (millis() - appState.lastActivityTime > INACTIVITY_TIMEOUT_MS) {
    enterDeepSleep();
  }
}


void setup() {
  Serial.begin(115200);
  
  jsonMutex = xSemaphoreCreateMutex();

  pinMode(PIN_ALCOHOL, INPUT);
  pinMode(PIN_TEMP, INPUT);
  pinMode(PIN_MIC, INPUT);
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_VIB, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  
  updateActivity();
  
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_GPIO || cause == ESP_SLEEP_WAKEUP_EXT1) {
  }

  alcoholSensor.reset();

  Wire.begin(PIN_SDA, PIN_SCL);
  if (!battery.begin()) {
    Serial.println("MAX17048 not found!");
  } else {
    Serial.println("MAX17048 init!");
  }

  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_logisoso28_tf);
  drawText("READY");

  setupRGB();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  LittleFS.begin(true);
  loadAllData();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    if(LittleFS.exists("/index.html")) req->send(LittleFS, "/index.html", "text/html");
    else req->send(200, "text/plain", "Breathalyzer Ready (No index.html)");
  });
  server.on("/chart.min.js", HTTP_GET, [](AsyncWebServerRequest *req){
    if(LittleFS.exists("/chart.min.js")) req->send(LittleFS, "/chart.min.js", "application/javascript");
    else req->send(404);
  });
  server.on("/chart-adapter.min.js", HTTP_GET, [](AsyncWebServerRequest *req){
    if(LittleFS.exists("/chart-adapter.min.js")) req->send(LittleFS, "/chart-adapter.min.js", "application/javascript");
    else req->send(404);
  });
  server.on("/log/download", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(LittleFS, FILE_MEASUREMENTS, "text/json");
  });

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("Server started");
}

bool isBlowing = false;
bool isStabilizing = false;
bool stabilizationStarted = false;
unsigned long stabilizationStartTime = 0;
unsigned long blowStartTime = 0;
unsigned long lastGoodBlowTime = 0;

void loop() {
  
  checkDeepSleepConditions();
  
  if (millis() - appState.lastBattSend > 10000 && !isStabilizing && !isBlowing) {
    sendBatteryWS();
    appState.lastBattSend = millis();
  }

  int micVal = analogRead(PIN_MIC);
  
  if (micVal > MIC_THRESHOLD && !isBlowing && !isStabilizing) {
    blowStartTime = millis();
    lastGoodBlowTime = millis();
    Serial.println("Blow detected!");
    
    updateActivity();
    analogWrite(PIN_BUZZER, 80);
    
    isBlowing = true;
  }

  if (isBlowing && !isStabilizing) {
    if (micVal > MIC_THRESHOLD) {
      lastGoodBlowTime = millis();
    }

    if ((millis() - lastGoodBlowTime) > BLOW_INTERRUPTION_TOLERANCE_MS && micVal <= MIC_THRESHOLD) {
      Serial.println("Measurement failed (Interrupted)!");
      analogWrite(PIN_BUZZER, 0); 
      isBlowing = false;
      drawText("RETRY");
      delay(1000); 
      drawText("READY");
    }
      
    else if (millis() - blowStartTime >= BLOW_TIME_REQUIRED_MS) {
      Serial.println("Blow complete -> Stabilizing...");
      analogWrite(PIN_BUZZER, 0);
      
      vibMotor.start(); 
      
      isStabilizing = true;
      stabilizationStarted = false;
      isBlowing = false;
      alcoholSensor.reset(); 
    }
  }
  
  if (isStabilizing) {
    updateActivity(); 

    if (!stabilizationStarted) {
      stabilizationStartTime = millis();
      stabilizationStarted = true;
    }

    if (millis() - stabilizationStartTime < 1500) {
       drawText("WAIT...");
    } else {
      drawText("...");
      
      float finalPromille = 0.0;
      
      if (alcoholSensor.update(PIN_ALCOHOL, finalPromille)) {
        
        Serial.printf("Sensor Stable! Result: %.2f promille\n", finalPromille);
        vibMotor.start(); 
        drawText(String(finalPromille, 2)); 
        sendMeasurementToWeb(finalPromille);
        
        unsigned long resultShowStart = millis();
        while(millis() - resultShowStart < 5000) {
            vibMotor.update();
            delay(10);         
        }

        sensorReset.start(VOFFSET);
        
        isStabilizing = false;
        stabilizationStarted = false;
      }
    }
  }
  
  vibMotor.update();

  if (sensorReset.isActive()) {
    if (sensorReset.update(PIN_ALCOHOL)) {
      Serial.println("Sensor ready for next!");
      drawText("READY");
    }
  }
}
