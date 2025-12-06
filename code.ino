#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_MAX1704X.h>

Adafruit_MAX17048 battery;

// --- RGB LED Pins (ändra till dina pins) ---
const int RED_PIN = A0;
const int GREEN_PIN = A1;
const int BLUE_PIN = A2;
const int alk_pin = 5;
const int temp_pin = 9;
const int mic_pin = 10;
const int knapp_pin = A4;
const int vib_Pin = A3;
const int buzz_pin = 6;
const int sda = SDA;
const int scl = SCL;

const float VREF_S3 = 3.3;        
const int ADC_MAX_S3 = 4095;
const float MV_PER_PPM = 0.88;    // mV/ppm Justera efter behov
const int Voffset = 1820; // Offset på alkoholsensorn i bitspänning

// Mätsekvens
bool hoppa=false;
bool hoppatwo=false;
unsigned long hopptid;

// Reset state
bool resetActive = false;
unsigned long resetStartTime = 0;
int resetBaseline = 0;
int resetTolerance = 0;
unsigned long resetTimeoutMs = 0;

// Konstanter för deep sleep hantering
const unsigned long INACTIVITY_TIMEOUT = 180000;  // 3 minuter i millisekunder
const unsigned long BUTTON_HOLD_TIME = 3000;      // 3 sekunder
unsigned long lastActivityTime = 0;
unsigned long buttonPressStartTime = 0;
bool buttonWasPressed = false;
unsigned long lastBattSend = 0;

int thresh_mic=1500;
char* utskrift;
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
unsigned long starttime = 0;
unsigned long villkor = 0;

// Loop
bool visat = false;

// Vib
bool vibRun = false;
unsigned long t0;
int stepCount;

// --- Server ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- WiFi ---
const char* ssid = "ESP32S3";
const char* password = "12345678";

// --- Filer ---
const char* USERS_FILE      = "/users.json";
const char* MEASURE_FILE    = "/measurements.json";
const char* ACTIVEUSER_FILE = "/active.txt";

// --- State i RAM ---
String usersJson        = "[]";
String measurementsJson = "[]";
String activeUserId     = "";


bool updateSensorReset(int pin) {
  if (!resetActive) return false;

  int v = analogRead(pin);

  // Är vi nära baseline?
  if (abs(v - resetBaseline) <= resetTolerance) {
    resetActive = false;
    Serial.printf("Sensor reset OK! ADC=%d (baseline=%d)\n", v, resetBaseline);
    return true;
  }

  // Timeout?
  if (millis() - resetStartTime >= resetTimeoutMs) {
    resetActive = false;
    Serial.printf("Sensor reset TIMEOUT! Fast på ADC=%d\n", v);
    return true;
  }

  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    lastDebug = millis();
    Serial.printf("Väntar på sensor-reset... ADC=%d (mål:%d)\n", v, resetBaseline);
  }

  return false;
}

void startSensorReset(int baseline, int tolerance = 30, unsigned long timeout = 8000) {
  resetActive = true;
  resetStartTime = millis();
  resetBaseline = baseline;
  resetTolerance = tolerance;
  resetTimeoutMs = timeout;

  rita("RESET...");
  Serial.println("Startar sensor-reset...");
}

void startVibration() {
  vibRun = true;
  t0 = millis();
  stepCount = 0;
  digitalWrite(vib_Pin, HIGH);
}

void updateVibration() {
  if (!vibRun) return;

  if (millis() - t0 >= 100) {
    t0 = millis();
    stepCount++;

    digitalWrite(vib_Pin, stepCount % 2 == 0 ? HIGH : LOW);

    if (stepCount >= 6) {
      vibRun = false;
      digitalWrite(vib_Pin, LOW);
    }
  }
}

float Promille(int rawValue) {
  float voltage = ((rawValue-Voffset) * VREF_S3) / ADC_MAX_S3;   
  float mV = voltage * 1000.0;
  float ppm = mV / MV_PER_PPM;
  float promille = ppm / 450.0;
  promille = round(promille * 10.0) / 10.0;
  if (promille < 0) promille = 0;

  return promille;
}

void updateActivity() {
  lastActivityTime = millis();
}

void rita(const String& text){
  display.clearBuffer();
  int width = display.getStrWidth(text.c_str());
  int len = (128 - width) / 2;
  display.drawStr(len, 50, text.c_str());
  display.sendBuffer();
}


void enterDeepSleep() {
  Serial.println("Går in i deep sleep...");
  
  // Släck RGB LED
  setRGBColor(0, 0, 0);
  
  // Rensa display
  rita("Zzz...");

  delay(1000);  // Visa meddelande 1 sekund
  display.clear();
  
  // Stäng av WiFi för att spara ström
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  // Konfigurera wake-up på knapp (A5 som EXT1)
  // EXT0 vaknar på LOW (knapp nedtryckt)
  esp_sleep_enable_ext1_wakeup(1ULL << knapp_pin, ESP_EXT1_WAKEUP_ALL_LOW);
  
  Serial.println("Deep sleep aktiverad. Tryck knapp för att väcka.");
  delay(100);

  // Gå in i deep sleep
  esp_deep_sleep_start();
}

void checkDeepSleep() {
  bool buttonPressed = (digitalRead(knapp_pin) == LOW);

  // Knapp just nedtryckt
  if (buttonPressed && !buttonWasPressed) {
    buttonPressStartTime = millis();
    updateActivity();
    buttonWasPressed = true;
  }

  // Knappen fortfarande nedtryckt
  if (buttonPressed && buttonWasPressed) {
    // HÅLLT INNE → deep sleep
    if (millis() - buttonPressStartTime >= BUTTON_HOLD_TIME) {
      Serial.println("Knapp hållen i 3 sekunder - deep sleep");
      enterDeepSleep();
    }
  }

  // Knappen släppt → kort tryck
  if (!buttonPressed && buttonWasPressed) {
    unsigned long held = millis() - buttonPressStartTime;
    buttonWasPressed = false;

    if (held < BUTTON_HOLD_TIME) {
      Serial.println("Kort knapptryck → visa batteri");
      showBattery();
    }
  }

  // Inaktivitet → deep sleep
  if (millis() - lastActivityTime > INACTIVITY_TIMEOUT) {
    Serial.println("Inaktiv i 3 minuter → deep sleep");
    enterDeepSleep();
  }
}

// Hantera uppvakning från deep sleep
void handleWakeup() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {

    case ESP_SLEEP_WAKEUP_EXT1: {
    rita("READY");
    }
    break;

    default:
      Serial.println("Normal start (ej från deep sleep)");
      break;
  }

  // Återställ inaktivitetstid
  updateActivity();
}

void sendMeasurementToWeb(float promilleValue) {
  if (!activeUserId || activeUserId == "") {
    Serial.println("Ingen aktiv användare - kan ej skicka mätning");
    return;
  }
  
  // Skapa mätningsobjekt med rå-timestamp (millis)
  StaticJsonDocument<200> measurementDoc;
  measurementDoc["type"] = "add_measurement";
  measurementDoc["value"] = promilleValue;
  measurementDoc["timestamp"] = millis();
  measurementDoc["userId"] = activeUserId;
  
  // Serialisera till JSON-sträng
  String jsonString;
  serializeJson(measurementDoc, jsonString);
  
  // Skicka till alla anslutna WebSocket-klienter
  ws.textAll(jsonString);
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

void setupRGB() {
  ledcAttach(RED_PIN, 5000, 8);   // Pin, frekvens, resolution
  ledcAttach(GREEN_PIN, 5000, 8);
  ledcAttach(BLUE_PIN, 5000, 8);
  setRGBColor(255, 255, 255);
}

void setRGBColor(int r, int g, int b) {
  ledcWrite(RED_PIN, 255 - r);
  ledcWrite(GREEN_PIN, 255 - g);
  ledcWrite(BLUE_PIN, 255 - b);
}

// Konvertera hex-färg (#RRGGBB) till RGB-värden
void setRGBFromHex(String hexColor) {
  // Ta bort # om den finns
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
  
  Serial.printf("RGB LED: #%s -> R:%d G:%d B:%d\n", hexColor.c_str(), r, g, b);
}

// Uppdatera LED-färg baserat på aktiv användare
void updateLEDForActiveUser() {
  if (activeUserId == "") {
    // Ingen aktiv användare - släck LED
    setRGBColor(0, 0, 0);
    Serial.println("RGB LED: Ingen aktiv användare, släcker LED");
    return;
  }
  
  // Hitta användarens färg
  DynamicJsonDocument users(8192);
  deserializeJson(users, usersJson);
  
  JsonArray arr = users.as<JsonArray>();
  for (JsonObject user : arr) {
    String id = user["id"] | "";
    if (id == activeUserId) {
      String color = user["color"] | "#000000";
      setRGBFromHex(color);
      Serial.printf("RGB LED: Aktiv användare: %s, färg: %s\n", 
                    user["name"].as<String>().c_str(), 
                    color.c_str());
      return;
    }
  }
  
  // Användare hittades inte - släck LED
  setRGBColor(0, 0, 0);
  Serial.println("RGB LED: Användare ej hittad, släcker LED");
}

// File helpers
String readFile(const char* path, const String& defVal = "[]") {
  if (!LittleFS.exists(path)) return defVal;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return defVal;
  String out = f.readString();
  f.close();
  out.trim();
  if (out == "") return defVal;
  return out;
}

void writeFile(const char* path, const String& data) {
  File f = LittleFS.open(path, FILE_WRITE);
  if (!f) return;
  f.print(data);
  f.close();
}

void writeActiveUser() {
  File f = LittleFS.open(ACTIVEUSER_FILE, FILE_WRITE);
  if (!f) return;
  f.print(activeUserId);
  f.close();
}

void loadAllData() {
  usersJson        = readFile(USERS_FILE, "[]");
  measurementsJson = readFile(MEASURE_FILE, "[]");

  if (LittleFS.exists(ACTIVEUSER_FILE)) {
    File f = LittleFS.open(ACTIVEUSER_FILE, FILE_READ);
    activeUserId = f.readString();
    activeUserId.trim();
    f.close();
  } else {
    activeUserId = "";
  }
  
  // Uppdatera LED efter data laddats
  updateLEDForActiveUser();
}

// WEBSOCKET BROADCAST
void broadcastState() {
  DynamicJsonDocument users(8192);
  DynamicJsonDocument meas(16384);
  DynamicJsonDocument root(32768);

  deserializeJson(users, usersJson);
  deserializeJson(meas, measurementsJson);

  root["type"] = "state";
  root["users"] = users.as<JsonArray>();
  root["measurements"] = meas.as<JsonArray>();
  root["activeUserId"] = activeUserId;

  String json;
  serializeJson(root, json);
  ws.textAll(json);
}

// WEBSOCKET HANDLER
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    // Skicka state direkt
    broadcastState();
    return;
  }

  if (type != WS_EVT_DATA) return;

  String msg = "";
  for (size_t i = 0; i < len; i++) msg += (char)data[i];

  DynamicJsonDocument doc(2048);
  deserializeJson(doc, msg);

  String t = doc["type"] | "";

  // ----------------------------------------
  // add_user
  // ----------------------------------------
  if (t == "add_user") {
    DynamicJsonDocument users(8192);
    deserializeJson(users, usersJson);

    JsonArray arr = users.as<JsonArray>();
    arr.add(doc["user"]);

    serializeJson(arr, usersJson);
    writeFile(USERS_FILE, usersJson);

    broadcastState();
  }

  // ----------------------------------------
  // set_active_user
  // ----------------------------------------
  else if (t == "set_active_user") {
    activeUserId = doc["activeUserId"].as<String>();
    writeActiveUser();
    
    // Uppdatera LED-färg
    updateLEDForActiveUser();
    
    broadcastState();
  }

  // ----------------------------------------
  // add_measurement
  // ----------------------------------------
  else if (t == "add_measurement") {
    DynamicJsonDocument meas(16384);
    deserializeJson(meas, measurementsJson);
    JsonArray arr = meas.as<JsonArray>();

    JsonObject m = arr.createNestedObject();
    m["timestamp"] = millis();
    m["userId"]    = activeUserId;
    m["value"]     = doc["value"];

    serializeJson(arr, measurementsJson);
    writeFile(MEASURE_FILE, measurementsJson);

    broadcastState();
  }

  // ----------------------------------------
  // clear_all
  // ----------------------------------------
  else if (t == "clear_all") {
    usersJson = "[]";
    measurementsJson = "[]";
    activeUserId = "";

    writeFile(USERS_FILE, usersJson);
    writeFile(MEASURE_FILE, measurementsJson);
    writeActiveUser();

    // Släck LED när allt rensas
    setRGBColor(0, 0, 0);

    broadcastState();
  }
}

// serverfile
void serveFile(AsyncWebServerRequest *req, const char* path, const char* type) {
  if (!LittleFS.exists(path)) {
    req->send(404, "text/plain", "File not found");
    return;
  }
  req->send(LittleFS, path, type);
}

void showBattery() {
  float percent = battery.cellPercent();
  float voltage = battery.cellVoltage();
  
  updateActivity();

  display.clearBuffer();
  display.setFont(u8g2_font_logisoso28_tf);

  String text = String((int)percent) + "%";
  int w = display.getStrWidth(text.c_str());
  int x = (128 - w) / 2;

  display.drawStr(x, 40, text.c_str());
  display.sendBuffer();

  Serial.printf("Batteri: %.2f%%  (%.2f V)\n", percent, voltage);

  delay(2000);   // Visa i 2 sekunder
  display.clearBuffer();
  display.sendBuffer();
}

bool stabilSensor(int pin, int &meanOut) {
  static int sampleCount = 0;
  static long sum = 0;
  static int minV = 99999;
  static int maxV = -99999;
  static unsigned long lastSampleTime = 0;

  if (millis() - lastSampleTime < 4) {
    return false;  
  }
  lastSampleTime = millis();

  int v = analogRead(pin);
  sum += v;
  if (v < minV) minV = v;
  if (v > maxV) maxV = v;

  sampleCount++;

  // Inte klar än → fortsätt samla
  if (sampleCount < 50) {
    return false;
  }

  // Alla samples klara → räkna ut medel & variation
  meanOut = sum / sampleCount;
  int variation = maxV - minV;

  sampleCount = 0;
  sum = 0;
  minV = 99999;
  maxV = -99999;

  return (variation < 25);
}

void resetStabilSensor() {
  // Tvinga fram en "reset" genom att anropa stabilSensor 
  // tills den är klar med felaktiga data
  int dummy;
  for(int i = 0; i < 100; i++) {
    stabilSensor(alk_pin, dummy);
  }
}

// Setup
void setup() {
  Serial.begin(115200);

  pinMode(alk_pin,INPUT);
  pinMode(temp_pin,INPUT);lastActivityTime = millis();
  handleWakeup();
  pinMode(mic_pin,INPUT);
  pinMode(knapp_pin,INPUT);
  pinMode(vib_Pin,OUTPUT);
  pinMode(buzz_pin,OUTPUT);

  resetStabilSensor();

  Wire.begin(sda,scl);

    if (!battery.begin()) {
  Serial.println("MAX17048 hittades inte!");
  } else {
  Serial.println("MAX17048 initierad!");
  }

  display.begin();
  display.clearBuffer();
  const char* text = "READY";
  int textWidth = display.getStrWidth(text);
  int x = (128 - textWidth) / 2;
  display.drawStr(x, 40, text);
  display.sendBuffer();


  // Initiera RGB LED
  setupRGB();
  Serial.println("RGB LED initierad");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  LittleFS.begin(true);
  loadAllData();

  // Static files
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
    serveFile(req, "/index.html", "text/html");
  });
  server.on("/chart.min.js", HTTP_GET, [](AsyncWebServerRequest *req){
    serveFile(req, "/chart.min.js", "application/javascript");
  });
  server.on("/chart-adapter.min.js", HTTP_GET, [](AsyncWebServerRequest *req){
    serveFile(req, "/chart-adapter.min.js", "application/javascript");
  });

  // CSV download
  server.on("/log/download", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(LittleFS, MEASURE_FILE, "text/json");
  });

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.begin();
  Serial.println("Server startad");
}

void loop() {

  // Kolla deep sleep
  checkDeepSleep();

  // Skicka batteri var 10:e sekund
  if (millis() - lastBattSend > 10000 && !hoppa) {
    sendBatteryWS();
    lastBattSend = millis();
  }

  int alk_value = analogRead(alk_pin);
  int mic_value = analogRead(mic_pin);

  if (mic_value > thresh_mic && !visat && !hoppa) {
    starttime = millis();
    villkor = millis();
    Serial.println("Blås upptäckt!");
    updateActivity();
    digitalWrite(buzz_pin, HIGH);
    visat = true;
  }

  if (visat && !hoppa) {

    if (mic_value > thresh_mic) {
      villkor = millis();
    }

    if ((millis() - villkor) > 250 && mic_value < thresh_mic) {
      Serial.println("Mätning misslyckad!");
      digitalWrite(buzz_pin, LOW);
      visat = false;
    }

    if (millis() - starttime >= 6000) {
      Serial.println("Blåsning klar → stabiliserar sensor...");
      digitalWrite(buzz_pin, LOW);

      startVibration();

      hoppa = true;        
      hoppatwo = false;    
      visat = false;       
    }
  }

  if (hoppa) {

    if (!hoppatwo) {
      hopptid = millis();
      hoppatwo = true;
    }

    if (millis() - hopptid < 2000) {
      return;
    }

    int medel = 0;
    rita("...");

    if (stabilSensor(alk_pin, medel)) {

      Serial.printf("Sensor stabil! ADC:", medel);

      float prom = Promille(medel);
      Serial.printf("Promille:", prom);

      startVibration();
      rita(String(prom));
      sendMeasurementToWeb(prom);
      startSensorReset(Voffset);

      hoppa = false;
      hoppatwo = false;
      visat = false;

    } else {
      Serial.printf("Sensor ej stabil ännu...");
    }
  }

  updateVibration();

  if (resetActive && updateSensorReset(alk_pin)) {
    Serial.println("Sensorn är redo för nästa mätning!");
    rita("READY");
  }
}



