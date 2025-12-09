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

// --- Configuration Constants ---
const char* WIFI_SSID = "ESP32S3";
const char* WIFI_PASS = "12345678";

// Pin Definitions (ESP32-S3 specific recommended, verify with your board)
const int PIN_RED 	= A0; // Verify if using ADC pins or GPIO numbers. On S3, A0 is typically GPIO1.
const int PIN_GREEN = A1;
const int PIN_BLUE 	= A2;
const int PIN_VIB 	= A3;
const int PIN_BUTTON = A4; // Used for wakeup
const int PIN_ALCOHOL = 5;
const int PIN_BUZZER 	= 6;
const int PIN_TEMP 	= 9;
const int PIN_MIC 	= 10;
const int PIN_SDA 	= SDA;
const int PIN_SCL 	= SCL;

// Sensor Constants
const float VREF_S3 = 3.3; 			
const int ADC_MAX_S3 = 4095;
const float MV_PER_PPM = 0.88; 	 // mV/ppm - Adjust as needed
const int VOFFSET = 1805; 			 // Sensor offset in raw ADC

// Timing Constants
const unsigned long INACTIVITY_TIMEOUT_MS = 180000; // 3 minutes
const unsigned long BUTTON_HOLD_TIME_MS = 3000; 	 // 3 seconds
const int MIC_THRESHOLD = 1500;
const int BLOW_TIME_REQUIRED_MS = 6000;
const int BLOW_INTERRUPTION_TOLERANCE_MS = 250;

// Files
const char* FILE_USERS 			= "/users.json";
const char* FILE_MEASUREMENTS= "/measurements.json";
const char* FILE_ACTIVE_USER = "/active.txt";

// --- Global Objects ---
Adafruit_MAX17048 battery;
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Mutex for JSON state thread safety
SemaphoreHandle_t jsonMutex;

// --- Application State ---
struct AppState {
	String usersJson = "[]";
	String measurementsJson = "[]";
	String activeUserId = "";
	unsigned long lastActivityTime = 0;
	unsigned long lastBattSend = 0;
} appState;

// Helper to update activity timestamp
void updateActivity() {
	appState.lastActivityTime = millis();
}

// --- Display Helper ---
void drawText(const String& text) {
	display.clearBuffer();
	// Centering logic
  display.setFont(u8g2_font_logisoso28_tf);
	int width = display.getStrWidth(text.c_str());
	int x = (128 - width) / 2;
	// Assuming height is roughly 64, draw at y=50
	display.drawStr(x, 50, text.c_str());
	display.sendBuffer();
}

// --- RGB LED Control ---
void setRGBColor(int r, int g, int b) {
	// Common anode or cathode? Original code used (255 - r), implies Common Anode.
	ledcWrite(PIN_RED, 255 - r);
	ledcWrite(PIN_GREEN, 255 - g);
	ledcWrite(PIN_BLUE, 255 - b);
}

void setupRGB() {
	// ESP32 Arduino Core 3.x syntax
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

// --- File System Helpers ---
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

// --- WebSocket Handling ---
void broadcastState() {
	// Requires large buffers. Allocate on heap implicitly via DynamicJsonDocument
	DynamicJsonDocument doc(32768); // Root
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
			// LED update needs to happen outside mutex if possible or carefully
			// But we can just signal or let broadcast handle it? 
			// Better to call updateLEDForActiveUser() later or here (but verify recursion/deadlock).
			// updateLEDForActiveUser takes mutex, so we shouldn't call it while holding mutex.
		}
		else if (t == "add_measurement") {
			DynamicJsonDocument measDoc(16384);
			deserializeJson(measDoc, appState.measurementsJson);
			JsonArray arr = measDoc.as<JsonArray>();
			JsonObject m = arr.createNestedObject();
			m["timestamp"] = millis();
			m["userId"] 	= appState.activeUserId;
			m["value"] 		= doc["value"];

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
		// Create a temporary doc to send the single event
		StaticJsonDocument<200> eventDoc;
		eventDoc["type"] = "add_measurement";
		eventDoc["value"] = promilleValue;
		eventDoc["timestamp"] = millis();
		eventDoc["userId"] = appState.activeUserId;
		
		// Update local state storage as well (logic duplicated from onWsEvent for robustness)
		DynamicJsonDocument measDoc(16384);
		deserializeJson(measDoc, appState.measurementsJson);
		JsonArray arr = measDoc.as<JsonArray>();
		JsonObject m = arr.createNestedObject();
		m["timestamp"] = millis();
		m["userId"] 	= appState.activeUserId;
		m["value"] 		= promilleValue;
		
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

// --- Sensor Logic ---
class SensorStabilizer {
private:
	int sampleCount = 0;
	long sum = 0;
	int minV = 99999;
	int maxV = -99999;
	unsigned long lastSampleTime = 0;
public:
	void reset() {
		sampleCount = 0;
		sum = 0;
		minV = 99999;
		maxV = -99999;
		lastSampleTime = 0;
	}

	bool update(int pin, int &meanOut) {
		if (millis() - lastSampleTime < 4) return false;
		lastSampleTime = millis();

		int v = analogRead(pin);
		sum += v;
		if (v < minV) minV = v;
		if (v > maxV) maxV = v;
		sampleCount++;

		if (sampleCount < 50) return false;

		meanOut = sum / sampleCount;
		int variation = maxV - minV;
		
		// Reset for next batch
		sampleCount = 0; 
		sum = 0; 
		minV = 99999; 
		maxV = -99999;

		return (variation < 25);
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

// --- Calculation ---
float calculatePromille(int rawValue) {
	float voltage = ((rawValue - VOFFSET) * VREF_S3) / ADC_MAX_S3; 	
	float mV = voltage * 1000.0;
	float ppm = mV / MV_PER_PPM;
	float promille = ppm / 450.0; // Conversion factor assumption
	
	promille = round(promille * 100) / 100;
	if (promille < 0) promille = 0;
	return promille;
}

// --- Deep Sleep & Power ---
void enterDeepSleep() {
	Serial.println("Entering deep sleep...");
	setRGBColor(0, 0, 0);
	drawText("Zzz...");
	delay(1000);
	display.clear();
	
	WiFi.disconnect(true);
	WiFi.mode(WIFI_OFF);
	
	// ESP32-S3 Specific Wakeup (KORRIGERAD KOD - Använder numeriskt värde 0)
	// 0 = Low Level. Bytte från ESP_GPIO_WAKEUP_LEVEL_LOW till 0 för kompatibilitet.
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
	
	// Blocking delay replaced by state machine? 
	// For simplicity in this function, we keep delay but warn it blocks handling.
	delay(2000); 
	display.clearBuffer();
	display.sendBuffer();
}

void checkDeepSleepConditions() {
	bool pressed = (digitalRead(PIN_BUTTON) == LOW);
	
	static bool initialized = false;
	static bool ignoreCurrentPress = false;
	static bool wasPressed = false;
	static unsigned long pressStart = 0;
	
	// Initialize state on first run
	if (!initialized) {
		// If button is held during boot (e.g. used to wake up), ignore this press
		// so it doesn't immediately count towards the shutdown timer.
		if (pressed) {
			ignoreCurrentPress = true;
			wasPressed = true;
		}
		initialized = true;
	}

	if (pressed) {
		if (!wasPressed) {
			// New press detected
			pressStart = millis();
			wasPressed = true;
			ignoreCurrentPress = false; 
			updateActivity();
		}
		
		// If this is a valid new press (not the one from boot)
		if (!ignoreCurrentPress) {
			// Check for long hold -> Deep Sleep
			if (millis() - pressStart >= BUTTON_HOLD_TIME_MS) {
				enterDeepSleep();
			}
		}
	} else {
		// Button released
		if (wasPressed) {
			// It was pressed, now released
			if (!ignoreCurrentPress) {
				unsigned long held = millis() - pressStart;
				if (held < BUTTON_HOLD_TIME_MS) {
					showBattery();
				}
			}
			// Reset state
			wasPressed = false;
			ignoreCurrentPress = false; 
		}
	}
	
	if (millis() - appState.lastActivityTime > INACTIVITY_TIMEOUT_MS) {
		enterDeepSleep();
	}
}

// --- Setup ---
void setup() {
	Serial.begin(115200);
	
	// Init Mutex
	jsonMutex = xSemaphoreCreateMutex();

	// Pins
	pinMode(PIN_ALCOHOL, INPUT);
	pinMode(PIN_TEMP, INPUT);
	pinMode(PIN_MIC, INPUT);
	pinMode(PIN_BUTTON, INPUT);
	pinMode(PIN_VIB, OUTPUT);
	pinMode(PIN_BUZZER, OUTPUT);
	
	// Reset activity timer
	updateActivity();
	
	// Check wakeup
	esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
	if (cause == ESP_SLEEP_WAKEUP_GPIO || cause == ESP_SLEEP_WAKEUP_EXT1) {
		// Just woke up
	}

	// Init Logic
	alcoholSensor.reset();

	// Wire & Battery
	Wire.begin(PIN_SDA, PIN_SCL);
	if (!battery.begin()) {
		Serial.println("MAX17048 not found!");
	} else {
		Serial.println("MAX17048 init!");
	}

	// Display
	display.begin();
	display.clearBuffer();
	// Font setup needed?
	display.setFont(u8g2_font_logisoso28_tf); // Set a default font
	drawText("READY");

	// RGB
	setupRGB();

	// WiFi
	WiFi.mode(WIFI_AP);
	WiFi.softAP(WIFI_SSID, WIFI_PASS);
	Serial.print("AP IP: ");
	Serial.println(WiFi.softAPIP());

	// FS
	LittleFS.begin(true);
	loadAllData();

	// Server
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

// --- Main Loop State ---
bool isBlowing = false;
bool isStabilizing = false;
bool stabilizationStarted = false;
unsigned long stabilizationStartTime = 0;
unsigned long blowStartTime = 0;
unsigned long lastGoodBlowTime = 0;

void loop() {
	checkDeepSleepConditions();

	// Battery Report
	if (millis() - appState.lastBattSend > 10000 && !isStabilizing) {
		sendBatteryWS();
		appState.lastBattSend = millis();
	}

	int micVal = analogRead(PIN_MIC);

	// 1. Detect Blow Start
	if (micVal > MIC_THRESHOLD && !isBlowing && !isStabilizing) {
		blowStartTime = millis();
		lastGoodBlowTime = millis();
		Serial.println("Blow detected!");
		updateActivity();
		analogWrite(PIN_BUZZER, 80);
		isBlowing = true;
	}

	// 2. Monitor Blow
	if (isBlowing && !isStabilizing) {
		if (micVal > MIC_THRESHOLD) {
			lastGoodBlowTime = millis();
		}

		// Interruption check
		if ((millis() - lastGoodBlowTime) > BLOW_INTERRUPTION_TOLERANCE_MS && micVal <= MIC_THRESHOLD) {
			Serial.println("Measurement failed (Interrupted)!");
			analogWrite(PIN_BUZZER, 0);
			isBlowing = false;
			drawText("RETRY");
		}
		// Completion check
		else if (millis() - blowStartTime >= BLOW_TIME_REQUIRED_MS) {
			Serial.println("Blow complete -> Stabilizing...");
			analogWrite(PIN_BUZZER, 0);
			vibMotor.start();
			
			isStabilizing = true;
			stabilizationStarted = false;
			isBlowing = false;
		}
	}

	// 3. Stabilization & Measurement
	if (isStabilizing) {
		if (!stabilizationStarted) {
			stabilizationStartTime = millis();
			stabilizationStarted = true;
			alcoholSensor.reset(); // Clear old buffer
		}

		// Wait a moment (2s) before reading?
		if (millis() - stabilizationStartTime < 2000) {
			 drawText("WAIT...");
			 // Continue loop to keep vibMotor updating
		} else {
			drawText("...");
			int meanVal = 0;
			if (alcoholSensor.update(PIN_ALCOHOL, meanVal)) {
				Serial.printf("Sensor Stable! ADC: %d\n", meanVal);
				float prom = calculatePromille(meanVal);
				Serial.printf("Promille: %.2f\n", prom);
				vibMotor.start();
				drawText(String(prom, 1)); // Display with 1 decimal
				sendMeasurementToWeb(prom);
				
				sensorReset.start(VOFFSET);
				
				isStabilizing = false;
				stabilizationStarted = false;
			} else {
				// Waiting for sensor to stabilize
			}
		}
	}

	vibMotor.update();

	if (sensorReset.isActive() && sensorReset.update(PIN_ALCOHOL)) {
		Serial.println("Sensor ready for next!");
		drawText("READY");
	}
}
