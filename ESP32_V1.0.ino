#include "RTC.h"
#include "Wire.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <TFT_eSPI.h>     // LCD display library

// Pin definitions for ESP32-WROOM
#define SDA_PIN 21        // I2C SDA pin (standard for ESP32)
#define SCL_PIN 22        // I2C SCL pin (standard for ESP32)
// #define DETECT_PIN 12     // OLD: Single box detection (replaced by 74HC165)
#define BL_PIN 19         // LCD backlight pin

// 74HC595 Shift Register pins for LED control
#define SR_DATA_PIN 27    // DS (Serial Data Input)
#define SR_LATCH_PIN 33   // STCP (Storage Register Clock, Latch)
#define SR_CLOCK_PIN 25   // SHCP (Shift Register Clock)

// 74HC165 Shift Register pins for Reed Switch input (7 boxes)
#define HC165_LOAD_PIN 14    // SH/LD̅ (Parallel Load)
#define HC165_CLOCK_PIN 32   // CLK (Shift Clock)
#define HC165_DATA_PIN 34    // Q̅H (Serial Data Output, input-only GPIO)

// Define LED_BUILTIN if not already defined (some ESP32 boards don't have it)
#ifndef LED_BUILTIN
#define LED_BUILTIN 5     // GPIO5 (changed from GPIO2 to avoid conflict with LCD RST)
#endif

// WiFi credentials - CHANGE THESE TO YOUR NETWORK
const char* ssid = "Verizon_3LG67L";          // Replace with your WiFi name
const char* password = "fluke4-rift-aha";   // Replace with your WiFi password

// TCP server configuration (must match Python system_config.json)
const int serverPort = 8080;
WiFiServer server(serverPort);
WiFiClient pythonClient;  // Connected Python client

// UDP discovery configuration
const int UDP_DISCOVERY_PORT = 8888;
WiFiUDP udp;

StaticJsonDocument<512> scheduleDoc;

// 74HC165 box state tracking (7 boxes, bit 0-6)
// Bit=0: Box OPEN (reed switch open, magnet far)
// Bit=1: Box CLOSED (reed switch closed, magnet near)
byte lastBoxStates = 0x7F;   // Initial: all closed (0b01111111)
byte currentBoxStates = 0x7F;
unsigned long lastDebounceTime[7] = {0}; // Debounce timestamp for each box
const unsigned long debounceDelay = 50;  // 50ms software debounce

// LCD Display object
TFT_eSPI tft = TFT_eSPI();
unsigned long lastLCDUpdate = 0;

// LCD display state management
enum DisplayState { DISPLAY_WELCOME, DISPLAY_READY, DISPLAY_REMINDER, DISPLAY_BOX_STATUS };
DisplayState currentDisplay = DISPLAY_WELCOME;
int lastDisplayedMedicationCount = -1;  // Track medication count changes for display update
byte lastDisplayedBoxStates = 0xFF;     // Track box state changes for display update


void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial monitor

  Serial.println("\n=== ESP32 Medication Reminder System ===");

  pinMode(LED_BUILTIN, OUTPUT);  // Setup built-in LED for debugging

  // Initialize 74HC595 pins for LED control
  pinMode(SR_DATA_PIN, OUTPUT);
  pinMode(SR_LATCH_PIN, OUTPUT);
  pinMode(SR_CLOCK_PIN, OUTPUT);

  // Clear all LEDs on startup
  clearAllLEDs();
  Serial.println("74HC595 shift register initialized");

  // Initialize 74HC165 pins for 7-box Reed Switch detection
  pinMode(HC165_LOAD_PIN, OUTPUT);
  pinMode(HC165_CLOCK_PIN, OUTPUT);
  pinMode(HC165_DATA_PIN, INPUT);  // GPIO 34 is input-only
  digitalWrite(HC165_LOAD_PIN, HIGH);  // Idle state HIGH
  digitalWrite(HC165_CLOCK_PIN, LOW);  // Idle state LOW
  Serial.println("74HC165 shift register initialized for 7-box detection");

  // Read initial box states
  currentBoxStates = read74HC165();
  lastBoxStates = currentBoxStates;
  Serial.print("Initial box states (Bit=0:OPEN, Bit=1:CLOSED): 0b");
  Serial.println(currentBoxStates, BIN);
  for (int i = 1; i <= 7; i++) {
    bool isClosed = (currentBoxStates >> (i-1)) & 0x01;
    Serial.print("  Box ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(isClosed ? "CLOSED" : "OPEN");
  }

  // Initialize LCD display
  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, HIGH);  // Turn on backlight

  tft.init();
  tft.setRotation(1);  // Landscape mode (0=portrait, 1=landscape, 2=portrait_inv, 3=landscape_inv)
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);

  // Display welcome screen
  tft.setCursor(60, 40);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(3);
  tft.println("💊 Smart Pillbox 🏥");

  tft.setCursor(80, 100);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("Initializing...");

  Serial.println("LCD display initialized");

  // Setup I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Setup RTC clock
  rtcBegin();
  Serial.print("Current time: ");
  Serial.println(rtcGetTime());

  // Connect to WiFi
  connectToWiFi();

  // Start TCP server
  server.begin();
  Serial.print("TCP server started on port ");
  Serial.println(serverPort);
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  // Start UDP discovery listener
  udp.begin(UDP_DISCOVERY_PORT);
  Serial.print("UDP discovery listener started on port ");
  Serial.println(UDP_DISCOVERY_PORT);

  // Load default schedule (will be overwritten by Python if connected)
  const char* medicineJson = R"rawliteral(
  {
    "1": {"time":"05:00", "duration":60, "dose":1, "note":"Advil: Take with food."},
    "2": {"time":"21:00", "duration":30, "dose":2, "note":"Tylenol: Eat before taking!"},
    "3": {"time":"11:30", "duration":60, "dose":3, "note":"Allergy pills"}
  }
  )rawliteral";

  setSchedule(medicineJson);
  Serial.println("System ready!");

  // Display box status on LCD (default screen)
  displayBoxStatus();
}

void loop() {
  blink();                    // Keep built-in LED blinking
  setLedIfTime();             // Check and control medication reminder LEDs
  handleTCPClient();          // Handle Python communication
  detectOpenClose();          // Monitor pillbox open/close events
  handleUDPDiscovery();       // Handle UDP discovery requests
  updateBoxStatusDisplay();   // Update box status display (smart refresh)

  // === Debug: Print box states every 2 seconds ===
  static unsigned long lastDebugPrint = 0;
  if (millis() - lastDebugPrint > 2000) {  // Every 2 seconds
    Serial.print("[DEBUG] Box states: 0b");
    Serial.print(currentBoxStates, BIN);
    Serial.print(" | ");
    for (int i = 1; i <= 7; i++) {
      bool isClosed = (currentBoxStates >> (i-1)) & 0x01;
      Serial.print(i);
      Serial.print(":");
      Serial.print(isClosed ? "C" : "O");
      Serial.print(" ");
    }
    Serial.println();
    lastDebugPrint = millis();
  }

  delay(100);  // Small delay to prevent overwhelming the system
}

/*
Example input:
{
  "1": {
    "time": "5:00",
    "duration": "1 hour",
    "dose": 1,
    "note": "Advil: Take with food."
  },
  "2": {
    "time": "21:00",
    "duration": "30 mins",
    "dose": 2,
    "note": "Tylenol: Eat before taking!"
  },
  "3": {
    "time": "1:30",
    "duration": "1 hour",
    "dose": 1,
    "note": "Allergy pills"
  }
}
*/

// 74HC595 LED state tracking
// Each bit in ledStates represents one LED (bit 0 = Box 1, bit 1 = Box 2, etc.)
byte ledStates = 0;  // 8-bit state for up to 8 LEDs

// Function to update 74HC595 shift register with LED states
void updateShiftRegister() {
  // Pull latch low to start sending data
  digitalWrite(SR_LATCH_PIN, LOW);

  // Send the 8-bit data to shift register
  shiftOut(SR_DATA_PIN, SR_CLOCK_PIN, MSBFIRST, ledStates);

  // Pull latch high to update outputs
  digitalWrite(SR_LATCH_PIN, HIGH);
}

// Function to set individual LED state (boxNumber: 1-7, state: HIGH/LOW)
void setBoxLED(int boxNumber, bool state) {
  if (boxNumber < 1 || boxNumber > 7) {
    Serial.print("Error: Invalid box number ");
    Serial.println(boxNumber);
    return;
  }

  // Convert box number (1-7) to bit position (0-6)
  int bitPos = boxNumber - 1;

  // Update the bit in ledStates
  if (state == HIGH) {
    ledStates |= (1 << bitPos);  // Set bit to 1 (turn LED on)
  } else {
    ledStates &= ~(1 << bitPos); // Set bit to 0 (turn LED off)
  }

  // Update the shift register hardware
  updateShiftRegister();

  Serial.print("Box ");
  Serial.print(boxNumber);
  Serial.print(" LED -> ");
  Serial.println(state ? "ON" : "OFF");
}

// Function to turn off all LEDs
void clearAllLEDs() {
  ledStates = 0;
  updateShiftRegister();
  Serial.println("All LEDs cleared");
}

// ==================== 74HC165 Input Functions (7-Box Detection) ====================

// Read 7-bit box states from 74HC165 shift register
// Returns: byte with bits 0-6 representing Box 1-7 states
//   Bit = 0: Box OPEN (reed switch open, magnet far, 74HC14 output LOW)
//   Bit = 1: Box CLOSED (reed switch closed, magnet near, 74HC14 output HIGH)
byte read74HC165() {
  byte result = 0;

  // Step 1: Load parallel data into shift register (pulse SH/LD̅ LOW)
  digitalWrite(HC165_LOAD_PIN, LOW);
  delayMicroseconds(5);  // tPLH: Parallel load setup time
  digitalWrite(HC165_LOAD_PIN, HIGH);
  delayMicroseconds(5);  // tPHL: Parallel load hold time

  // Step 2: Read 8 bits serially (we use 7 bits for Box 1-7)
  for (int i = 0; i < 8; i++) {
    // Read data bit from Q̅H (GPIO 34)
    byte bit = digitalRead(HC165_DATA_PIN);

    // Shift and store (MSB first from 74HC165)
    result |= (bit << (7 - i));

    // Clock pulse to shift next bit
    digitalWrite(HC165_CLOCK_PIN, HIGH);
    delayMicroseconds(5);  // tCH: Clock high time
    digitalWrite(HC165_CLOCK_PIN, LOW);
    delayMicroseconds(5);  // tCL: Clock low time
  }

  // Mask to keep only lower 7 bits (Box 1-7)
  result &= 0x7F;

  return result;
}

// Legacy schedule structure (for backward compatibility)
struct Schedule {
  String time;
  int dose;  // Box number (1-7)
  int duration;
  String note;
  bool triggered;
  unsigned long triggerMillis;
};

Schedule schedules[5];
int scheduleCount = 0;

// New medication schedule structure (for SET_SCHEDULE command)
struct MedicationSchedule {
  String medication_id;        // M0-M9
  String medication_name;      // Full name
  int times_per_day;           // Number of times to take per day
  int pills_per_dose;          // Pills per dose
  String schedule_times[6];    // Up to 6 times per day
  int schedule_times_count;    // Actual number of times
  String start_date;           // YYYY-MM-DD
  String end_date;             // YYYY-MM-DD
  String notes;                // Instructions
  int duration;                // Alert duration in minutes
  bool is_active;              // Active status
  int assigned_box;            // Auto-assigned storage box (0-based)
  bool triggered[6];           // Triggered status for each time
  unsigned long triggerMillis[6]; // Trigger timestamps
};

MedicationSchedule medications[10];  // Support up to 10 medications (M0-M9)
int medicationCount = 0;

void setSchedule(const String& jsonInput) {
  scheduleDoc.clear();
  DeserializationError error = deserializeJson(scheduleDoc, jsonInput);

  if (error) {
    Serial.print("Invalid JSON: ");
    Serial.println(error.c_str());
    return;
  }

  JsonObject root = scheduleDoc.as<JsonObject>();
  scheduleCount = 0;

  for (JsonPair schedule : root) {
    if (scheduleCount >= 5) break; // max schedules
    JsonObject info = schedule.value().as<JsonObject>();

    schedules[scheduleCount].time = info["time"].as<const char*>();
    schedules[scheduleCount].dose = info["dose"].as<int>();
    schedules[scheduleCount].note = info["note"].as<const char*>();
    schedules[scheduleCount].duration = info["duration"].as<int>();

    // Validate box number (1-7)
    if (schedules[scheduleCount].dose < 1 || schedules[scheduleCount].dose > 7) {
      Serial.print("Warning: Invalid box number ");
      Serial.println(schedules[scheduleCount].dose);
      schedules[scheduleCount].dose = 1; // Default to box 1
    }

    schedules[scheduleCount].triggered = false;
    schedules[scheduleCount].triggerMillis = 0;

    // Serial.print("Schedule "); Serial.println(scheduleCount + 1);
    // Serial.print("  Time: "); Serial.println(schedules[scheduleCount].time);
    // Serial.print("  Dose: "); Serial.println(schedules[scheduleCount].dose);
    // Serial.print("  Note: "); Serial.println(schedules[scheduleCount].note);
    // Serial.print("  Duration: "); Serial.println(schedules[scheduleCount].duration);

    scheduleCount++;
  }
}

void setLedIfTime() {
  String currentTime = rtcGetTime();
  unsigned long now = millis();

  // Process new medication schedule format (if configured)
  if (medicationCount > 0) {
    for (int i = 0; i < medicationCount; i++) {
      if (!medications[i].is_active) continue;

      int boxIdx = medications[i].assigned_box;
      if (boxIdx < 0 || boxIdx >= 7) continue;

      // Check each scheduled time for this medication
      for (int j = 0; j < medications[i].schedule_times_count; j++) {
        String schedTime = medications[i].schedule_times[j];

        // Check if it's time to trigger the LED
        if (!medications[i].triggered[j] && currentTime == schedTime) {
          setBoxLED(boxIdx + 1, HIGH);  // boxIdx is 0-based, setBoxLED expects 1-based
          medications[i].triggered[j] = true;
          medications[i].triggerMillis[j] = now;

          Serial.print("[Reminder] ");
          Serial.print(medications[i].medication_name);
          Serial.print(" (");
          Serial.print(medications[i].medication_id);
          Serial.print(") - Box ");
          Serial.print(boxIdx);
          Serial.print(" at ");
          Serial.println(currentTime);
          Serial.print("  Take ");
          Serial.print(medications[i].pills_per_dose);
          Serial.println(" pill(s)");
          if (medications[i].notes.length() > 0) {
            Serial.print("  Note: ");
            Serial.println(medications[i].notes);
          }

          // Display medication reminder on LCD
          displayMedicationReminder(i);
        }

        // Check if duration has expired and turn off LED
        if (medications[i].triggered[j] &&
            now - medications[i].triggerMillis[j] >= (unsigned long)medications[i].duration * 60000) {
          setBoxLED(boxIdx + 1, LOW);  // boxIdx is 0-based, setBoxLED expects 1-based
          medications[i].triggered[j] = false;

          Serial.print("[Reminder End] ");
          Serial.print(medications[i].medication_id);
          Serial.print(" - Box ");
          Serial.println(boxIdx);

          // Clear reminder from LCD, return to box status screen
          displayBoxStatus();
        }
      }
    }
  }
  // Fallback to legacy schedule format (for backward compatibility)
  else if (scheduleCount > 0) {
    for (int i = 0; i < scheduleCount; i++) {
      // Check if it's time to trigger the LED
      if (!schedules[i].triggered && currentTime == schedules[i].time) {
        setBoxLED(schedules[i].dose, HIGH);
        schedules[i].triggered = true;
        schedules[i].triggerMillis = now;

        Serial.print("Reminder triggered for box ");
        Serial.print(schedules[i].dose);
        Serial.print(" at ");
        Serial.println(currentTime);

        // TODO: Display code
      }

      // Check if duration has expired and turn off LED
      if (schedules[i].triggered && now - schedules[i].triggerMillis >= (unsigned long)schedules[i].duration * 60000) {
        setBoxLED(schedules[i].dose, LOW);
        schedules[i].triggered = false;

        Serial.print("Reminder ended for box ");
        Serial.println(schedules[i].dose);
      }
    }
  }
}

// Detect medication box open/close events (7 boxes via 74HC165)
void detectOpenClose() {
  // Read current state of all 7 boxes from 74HC165
  currentBoxStates = read74HC165();

  // Check each box (1-7) for state changes
  for (int boxNum = 1; boxNum <= 7; boxNum++) {
    int bitIndex = boxNum - 1;  // Bit 0 = Box 1, Bit 6 = Box 7

    // Extract current and last state for this box
    bool currentClosed = (currentBoxStates >> bitIndex) & 0x01;  // Bit=1: CLOSED
    bool lastClosed = (lastBoxStates >> bitIndex) & 0x01;

    // Detect state change
    if (currentClosed != lastClosed) {
      lastDebounceTime[bitIndex] = millis();
    }

    // Software debounce: wait 50ms after state change
    if ((millis() - lastDebounceTime[bitIndex]) > debounceDelay) {
      // State has been stable for debounceDelay
      if (currentClosed != lastClosed) {
        // Update last state
        if (currentClosed) {
          lastBoxStates |= (1 << bitIndex);   // Set bit (CLOSED)
        } else {
          lastBoxStates &= ~(1 << bitIndex);  // Clear bit (OPEN)
        }

        // Handle state change event
        if (!currentClosed) {
          // Box OPENED (Bit changed from 1 to 0)
          Serial.print("Box ");
          Serial.print(boxNum);
          Serial.println(" opened");
          sendBoxEvent(boxNum, true);  // true = opened

          // Turn off LED if medication is active for this box
          for (int i = 0; i < medicationCount; i++) {
            // Check if this medication is assigned to the opened box (boxNum is 1-based, assigned_box is 0-based)
            if (medications[i].assigned_box == (boxNum - 1) && medications[i].is_active) {
              // Turn off LED for this box
              setBoxLED(boxNum, LOW);
              Serial.print("LED ");
              Serial.print(boxNum);
              Serial.println(" turned off (medication taken)");

              // Mark all triggered times for this medication as complete
              for (int j = 0; j < medications[i].schedule_times_count; j++) {
                medications[i].triggered[j] = false;
              }
              break;
            }
          }
        } else {
          // Box CLOSED (Bit changed from 0 to 1)
          Serial.print("Box ");
          Serial.print(boxNum);
          Serial.println(" closed");
          sendBoxEvent(boxNum, false);  // false = closed
        }
      }
    }
  }
}

// Debug helper function: Print current box states
void printBoxStates() {
  Serial.print("Box States (Bit=0:OPEN, Bit=1:CLOSED): 0b");
  Serial.println(currentBoxStates, BIN);
  for (int i = 1; i <= 7; i++) {
    bool isClosed = (currentBoxStates >> (i-1)) & 0x01;
    Serial.print("  Box ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(isClosed ? "CLOSED" : "OPEN");
  }
}

// Non-blocking blink function for built-in LED (debugging/power indicator)
unsigned long blinkMillis = 0;
const long interval = 1000;  // Blink interval in milliseconds
void blink() {
  unsigned long currentMillis = millis();

  if (currentMillis - blinkMillis >= interval) {
    blinkMillis = currentMillis;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

// ==================== Network Communication Functions ====================

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed!");
    Serial.println("System will continue with local mode only.");
  }
}

void handleTCPClient() {
  // Check for new client connection
  if (!pythonClient || !pythonClient.connected()) {
    pythonClient = server.available();
    if (pythonClient) {
      Serial.println("New Python client connected!");
      sendWelcomeMessage();
    }
  }

  // Handle existing client
  if (pythonClient && pythonClient.connected()) {
    if (pythonClient.available()) {
      String message = pythonClient.readStringUntil('\n');
      message.trim();

      if (message.length() > 0) {
        Serial.println("Received from Python: " + message);
        handlePythonCommand(message);
      }
    }
  }
}

void handlePythonCommand(const String& command) {
  // Parse command - check if JSON
  if (command.startsWith("{")) {
    // Parse JSON to check for "cmd" field
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, command);

    if (error) {
      Serial.print("JSON parse error: ");
      Serial.println(error.c_str());
      sendResponse("{\"status\":\"ERROR\",\"message\":\"Invalid JSON\"}");
      return;
    }

    // Check if this is a command with header
    if (doc.containsKey("cmd")) {
      String cmd = doc["cmd"].as<String>();

      if (cmd == "SET_SCHEDULE") {
        handleSetSchedule(doc);
      }
      else if (cmd == "GET_SCHEDULE") {
        handleGetSchedule();
      }
      else if (cmd == "SET_TIME") {
        handleSetTime(doc);
      }
      else {
        Serial.println("Unknown command: " + cmd);
        sendResponse("{\"status\":\"ERROR\",\"message\":\"Unknown command\"}");
      }
    }
    else {
      // Legacy JSON schedule format (for backward compatibility)
      Serial.println("Updating schedule from Python (legacy format)...");
      setSchedule(command);
      sendResponse("OK:Schedule updated");
    }
  }
  else if (command.startsWith("PING")) {
    sendResponse("PONG");
  }
  else if (command.startsWith("STATUS")) {
    sendStatusReport();
  }
  else if (command.startsWith("TIME")) {
    String currentTime = rtcGetTime();
    sendResponse("TIME:" + currentTime);
  }
  else {
    sendResponse("ERROR:Unknown command");
  }
}

void handleSetSchedule(JsonDocument& doc) {
  Serial.println("[SET_SCHEDULE] Processing medication schedule configuration...");

  if (!doc.containsKey("medications")) {
    sendResponse("{\"status\":\"ERROR\",\"message\":\"Missing medications array\"}");
    return;
  }

  JsonArray meds = doc["medications"].as<JsonArray>();
  medicationCount = 0;

  // Auto-assign medications to available boxes
  JsonArray assignments = doc.createNestedArray("assignments");

  for (JsonObject med : meds) {
    if (medicationCount >= 10) {
      Serial.println("[Warning] Maximum 10 medications supported, ignoring extras");
      break;
    }

    // Parse medication data
    medications[medicationCount].medication_id = med["medication_id"].as<String>();
    medications[medicationCount].medication_name = med["medication_name"].as<String>();
    medications[medicationCount].times_per_day = med["times_per_day"] | 1;
    medications[medicationCount].pills_per_dose = med["pills_per_dose"] | 1;
    medications[medicationCount].start_date = med["start_date"].as<String>();
    medications[medicationCount].end_date = med["end_date"].as<String>();
    medications[medicationCount].notes = med["notes"].as<String>();
    medications[medicationCount].duration = med["duration"] | 60;
    medications[medicationCount].is_active = med["is_active"] | true;

    // Parse schedule times
    JsonArray times = med["schedule_times"].as<JsonArray>();
    int timeIdx = 0;
    for (JsonVariant time : times) {
      if (timeIdx < 6) {
        medications[medicationCount].schedule_times[timeIdx] = time.as<String>();
        medications[medicationCount].triggered[timeIdx] = false;
        medications[medicationCount].triggerMillis[timeIdx] = 0;
        timeIdx++;
      }
    }
    medications[medicationCount].schedule_times_count = timeIdx;

    // Auto-assign to storage box (simple: assign sequentially to available boxes)
    medications[medicationCount].assigned_box = medicationCount % 7;  // Cycle through 7 boxes (0-6)

    // Log assignment
    Serial.print("  [");
    Serial.print(medications[medicationCount].medication_id);
    Serial.print("] ");
    Serial.print(medications[medicationCount].medication_name);
    Serial.print(" -> Box ");
    Serial.print(medications[medicationCount].assigned_box);
    Serial.print(" (");
    Serial.print(medications[medicationCount].times_per_day);
    Serial.println(" times/day)");

    // Add to assignments array for response
    JsonObject assignment = assignments.createNestedObject();
    assignment["medication_id"] = medications[medicationCount].medication_id;
    assignment["box"] = medications[medicationCount].assigned_box;

    medicationCount++;
  }

  // Send success response with assignments
  StaticJsonDocument<512> responseDoc;
  responseDoc["status"] = "OK";
  responseDoc["message"] = "Schedule configured successfully";
  responseDoc["medicationCount"] = medicationCount;

  JsonArray respAssignments = responseDoc.createNestedArray("assignments");
  for (int i = 0; i < medicationCount; i++) {
    JsonObject assignment = respAssignments.createNestedObject();
    assignment["medication_id"] = medications[i].medication_id;
    assignment["box"] = medications[i].assigned_box;
  }

  String response;
  serializeJson(responseDoc, response);
  sendResponse(response);

  Serial.print("[SET_SCHEDULE] Configuration complete. ");
  Serial.print(medicationCount);
  Serial.println(" medications loaded.");

  // Show assigned boxes with LED feedback
  showAssignedBoxes();

  // Update LCD to show box status with new medication count
  displayBoxStatus();
}

// Display assigned boxes by blinking LEDs sequentially
void showAssignedBoxes() {
  Serial.println("[LED] Indicating assigned boxes...");

  bool boxUsed[7] = {false};  // Track which boxes are assigned

  // Mark all assigned boxes
  for (int i = 0; i < medicationCount; i++) {
    int box = medications[i].assigned_box;
    if (box >= 0 && box < 7) boxUsed[box] = true;
  }

  // Blink each assigned box LED sequentially
  for (int box = 0; box < 7; box++) {
    if (boxUsed[box]) {
      setBoxLED(box + 1, HIGH);  // Turn on (box is 0-indexed, setBoxLED expects 1-7)
      delay(2000);                // Keep lit for 2 seconds
      setBoxLED(box + 1, LOW);   // Turn off
      delay(300);                 // Pause between boxes
    }
  }

  Serial.println("[LED] Assignment indication complete");
}

void handleGetSchedule() {
  Serial.println("[GET_SCHEDULE] Sending current medication configuration...");

  StaticJsonDocument<2048> responseDoc;
  responseDoc["status"] = "OK";
  responseDoc["medicationCount"] = medicationCount;

  JsonArray meds = responseDoc.createNestedArray("medications");
  for (int i = 0; i < medicationCount; i++) {
    JsonObject med = meds.createNestedObject();
    med["medication_id"] = medications[i].medication_id;
    med["medication_name"] = medications[i].medication_name;
    med["times_per_day"] = medications[i].times_per_day;
    med["pills_per_dose"] = medications[i].pills_per_dose;
    med["assigned_box"] = medications[i].assigned_box;
    med["is_active"] = medications[i].is_active;

    JsonArray times = med.createNestedArray("schedule_times");
    for (int j = 0; j < medications[i].schedule_times_count; j++) {
      times.add(medications[i].schedule_times[j]);
    }
  }

  String response;
  serializeJson(responseDoc, response);
  sendResponse(response);
}

void handleSetTime(JsonDocument& doc) {
  Serial.println("[SET_TIME] Processing time set request...");

  if (!doc.containsKey("datetime")) {
    sendResponse("{\"status\":\"ERROR\",\"message\":\"Missing datetime field\"}");
    return;
  }

  String datetime = doc["datetime"].as<String>();

  if (rtcSetTime(datetime)) {
    StaticJsonDocument<256> responseDoc;
    responseDoc["status"] = "OK";
    responseDoc["message"] = "Time set successfully";
    responseDoc["current_time"] = rtcGetTime();

    String response;
    serializeJson(responseDoc, response);
    sendResponse(response);

    Serial.println("[SET_TIME] Time set successfully");

    // Update LCD to show new time
    displayBoxStatus();
  } else {
    sendResponse("{\"status\":\"ERROR\",\"message\":\"Invalid datetime format. Use YYYY-MM-DD HH:MM:SS\"}");
  }
}

void sendResponse(const String& response) {
  if (pythonClient && pythonClient.connected()) {
    pythonClient.println(response);
    Serial.println("Sent to Python: " + response);
  }
}

void sendWelcomeMessage() {
  StaticJsonDocument<256> doc;
  doc["type"] = "welcome";
  doc["device"] = "ESP32-Pillbox";
  doc["ip"] = WiFi.localIP().toString();
  doc["time"] = rtcGetTime();
  doc["boxes"] = 7;

  String output;
  serializeJson(doc, output);
  sendResponse(output);
}

void sendStatusReport() {
  StaticJsonDocument<512> doc;
  doc["type"] = "status";
  doc["time"] = rtcGetTime();
  doc["wifi"] = (WiFi.status() == WL_CONNECTED);
  doc["scheduleCount"] = scheduleCount;

  JsonArray activeReminders = doc.createNestedArray("activeReminders");
  for (int i = 0; i < scheduleCount; i++) {
    if (schedules[i].triggered) {
      JsonObject reminder = activeReminders.createNestedObject();
      reminder["dose"] = schedules[i].dose;
      reminder["time"] = schedules[i].time;
    }
  }

  String output;
  serializeJson(doc, output);
  sendResponse(output);
}

void sendBoxEvent(int boxNumber, bool isOpen) {
  if (pythonClient && pythonClient.connected()) {
    StaticJsonDocument<256> doc;
    doc["type"] = "box_event";
    doc["box"] = boxNumber;
    doc["state"] = isOpen ? "open" : "closed";
    doc["time"] = rtcGetTime();

    String output;
    serializeJson(doc, output);
    sendResponse(output);
  }
}

// ==================== UDP Discovery Functions ====================

void handleUDPDiscovery() {
  int packetSize = udp.parsePacket();

  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);

    if (len > 0) {
      incomingPacket[len] = 0;  // Null terminate
      String request = String(incomingPacket);

      // Check if this is a discovery request
      if (request.indexOf("DISCOVER") >= 0 || request.indexOf("ESP32") >= 0) {
        Serial.print("Discovery request from: ");
        Serial.print(udp.remoteIP());
        Serial.print(":");
        Serial.println(udp.remotePort());

        // Send discovery response with device info
        sendUDPDiscoveryResponse();
      }
    }
  }
}

void sendUDPDiscoveryResponse() {
  StaticJsonDocument<256> doc;
  doc["device"] = "ESP32-Pillbox";
  doc["ip"] = WiFi.localIP().toString();
  doc["port"] = serverPort;
  doc["type"] = "discovery_response";
  doc["boxes"] = 7;
  doc["time"] = rtcGetTime();

  String response;
  serializeJson(doc, response);

  // Send response back to requester
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.print(response);
  udp.endPacket();

  Serial.println("Sent discovery response: " + response);
}

// ==================== LCD Display Functions ====================

void displaySystemReady() {
  // Only redraw if state changed OR medication count changed
  if (currentDisplay == DISPLAY_READY &&
      lastDisplayedMedicationCount == medicationCount) {
    return;  // Already showing system ready screen with same data, skip
  }

  Serial.println("[LCD] Switching to System Ready display");
  currentDisplay = DISPLAY_READY;
  lastDisplayedMedicationCount = medicationCount;  // Update displayed count

  tft.fillScreen(TFT_BLACK);

  // Title
  tft.setTextSize(3);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(60, 30);
  tft.println("Smart Pillbox");

  // Status
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(100, 90);
  tft.println("System Ready");

  // Medication count
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(40, 130);
  tft.print("Medications: ");
  tft.println(medicationCount);

  // WiFi status
  tft.setCursor(40, 160);
  tft.print("WiFi: ");
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Connected");
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("Disconnected");
  }

  // Current time will be updated by updateLCDStatus()
}

void displayMedicationReminder(int medIndex) {
  Serial.println("[LCD] Switching to Medication Reminder display");
  currentDisplay = DISPLAY_REMINDER;

  tft.fillScreen(TFT_BLACK);

  // Alert title
  tft.setTextSize(3);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(40, 10);
  tft.println("TIME TO TAKE!");

  // Medication name
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 60);

  // Truncate long names if needed
  String medName = medications[medIndex].medication_name;
  if (medName.length() > 20) {
    medName = medName.substring(0, 17) + "...";
  }
  tft.println(medName);

  // Medication ID
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 90);
  tft.print("ID: ");
  tft.println(medications[medIndex].medication_id);

  // Box number
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(10, 120);
  tft.print("Open Box: ");
  tft.setTextSize(3);
  tft.println(medications[medIndex].assigned_box);

  // Pills per dose
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 160);
  tft.print("Take ");
  tft.print(medications[medIndex].pills_per_dose);
  tft.print(" pill");
  if (medications[medIndex].pills_per_dose > 1) {
    tft.print("s");
  }

  // Notes (instructions)
  if (medications[medIndex].notes.length() > 0) {
    tft.setTextSize(1);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.setCursor(10, 200);
    tft.println("Instructions:");

    // Word wrap for notes
    String notes = medications[medIndex].notes;
    int lineY = 215;
    int maxCharsPerLine = 40;
    int startIdx = 0;

    while (startIdx < notes.length() && lineY < 230) {
      int endIdx = startIdx + maxCharsPerLine;
      if (endIdx > notes.length()) {
        endIdx = notes.length();
      }

      // Try to break at space
      if (endIdx < notes.length()) {
        int spaceIdx = notes.lastIndexOf(' ', endIdx);
        if (spaceIdx > startIdx) {
          endIdx = spaceIdx;
        }
      }

      tft.setCursor(10, lineY);
      tft.println(notes.substring(startIdx, endIdx));

      startIdx = endIdx + 1;
      lineY += 15;
    }
  }
}

// ==================== LCD Box Status Display Module ====================

// Draw a single box icon (enlarged and prominent)
void drawBoxIcon(int x, int y, bool isClosed, bool hasAlert) {
  // isClosed: true=closed (filled square), false=open (empty square)
  // hasAlert: true=has medication reminder (yellow highlight)

  uint16_t boxColor;
  if (hasAlert) {
    boxColor = TFT_YELLOW;  // Alert: yellow
  } else if (isClosed) {
    boxColor = TFT_GREEN;   // Closed: green
  } else {
    boxColor = TFT_RED;     // Open: red
  }

  int boxSize = 30;  // 30x30 pixels (large and prominent)

  if (isClosed || hasAlert) {
    // Filled square ■
    tft.fillRect(x, y, boxSize, boxSize, boxColor);
    // Add border for better visibility
    tft.drawRect(x, y, boxSize, boxSize, TFT_WHITE);
  } else {
    // Empty square □
    tft.drawRect(x, y, boxSize, boxSize, boxColor);
    tft.drawRect(x+1, y+1, boxSize-2, boxSize-2, boxColor);  // Double border
  }
}

// Draw box status text below icon
void drawBoxStatusText(int x, int y, bool isClosed, bool hasAlert) {
  tft.setTextSize(1);

  uint16_t textColor;
  String statusText;

  if (hasAlert) {
    textColor = TFT_YELLOW;
    statusText = "ALERT";
  } else if (isClosed) {
    textColor = TFT_GREEN;
    statusText = "OK";
  } else {
    textColor = TFT_RED;
    statusText = "OPEN";
  }

  tft.setTextColor(textColor, TFT_BLACK);
  tft.setCursor(x, y);
  tft.print(statusText);
}

// Count open and closed boxes
void countBoxStates(byte boxStates, int &openCount, int &closedCount) {
  openCount = 0;
  closedCount = 0;

  for (int i = 0; i < 7; i++) {
    if ((boxStates >> i) & 0x01) {
      closedCount++;  // Bit=1: CLOSED
    } else {
      openCount++;    // Bit=0: OPEN
    }
  }
}

// Check if a box has an active medication alert
bool isBoxHighlighted(int boxNum) {
  for (int i = 0; i < medicationCount; i++) {
    if (medications[i].assigned_box == (boxNum - 1) && medications[i].is_active) {
      // Check if any time is triggered
      for (int j = 0; j < medications[i].schedule_times_count; j++) {
        if (medications[i].triggered[j]) {
          return true;
        }
      }
    }
  }
  return false;
}

// Main display function: Show box status screen
void displayBoxStatus() {
  Serial.println("[LCD] Displaying box status");
  currentDisplay = DISPLAY_BOX_STATUS;

  tft.fillScreen(TFT_BLACK);

  // === Top: Large Time Display ===
  tft.setTextSize(4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(30, 10);
  tft.print(rtcGetTime());

  // === WiFi Status + Medication Count ===
  tft.setTextSize(2);
  tft.setCursor(10, 50);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print("WiFi:");

  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("ON");
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("OFF");
  }

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.print(" Med:");
  tft.print(medicationCount);

  // === Separator Line ===
  tft.drawLine(0, 75, 240, 75, TFT_DARKGREY);

  // === Draw 7 Boxes (2 rows: 4 + 3) ===
  int startX = 15;
  int spacing = 55;

  // Row 1: Boxes 1-4
  int row1Y = 90;
  for (int i = 1; i <= 4; i++) {
    int x = startX + (i - 1) * spacing;

    bool isClosed = (currentBoxStates >> (i - 1)) & 0x01;
    bool hasAlert = isBoxHighlighted(i);

    // Box number
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(x + 10, row1Y);
    tft.print(i);

    // Box icon
    drawBoxIcon(x, row1Y + 20, isClosed, hasAlert);

    // Status text
    drawBoxStatusText(x + 5, row1Y + 55, isClosed, hasAlert);
  }

  // Row 2: Boxes 5-7
  int row2Y = 170;
  for (int i = 5; i <= 7; i++) {
    int x = startX + (i - 5) * spacing;

    bool isClosed = (currentBoxStates >> (i - 1)) & 0x01;
    bool hasAlert = isBoxHighlighted(i);

    // Box number
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(x + 10, row2Y);
    tft.print(i);

    // Box icon
    drawBoxIcon(x, row2Y + 20, isClosed, hasAlert);

    // Status text
    drawBoxStatusText(x + 5, row2Y + 55, isClosed, hasAlert);
  }

  // === Separator Line ===
  tft.drawLine(0, 260, 240, 260, TFT_DARKGREY);

  // === Statistics (Large and Prominent) ===
  int openCount, closedCount;
  countBoxStates(currentBoxStates, openCount, closedCount);

  tft.setTextSize(3);
  tft.setCursor(10, 275);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.print("Open:");
  tft.print(openCount);

  tft.setCursor(130, 275);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print("OK:");
  tft.print(closedCount);

  lastDisplayedBoxStates = currentBoxStates;
  lastDisplayedMedicationCount = medicationCount;
}

// Update box status display (only redraw if state changed)
void updateBoxStatusDisplay() {
  // Redraw entire screen if box states or medication count changed
  if (currentDisplay == DISPLAY_BOX_STATUS &&
      (currentBoxStates != lastDisplayedBoxStates ||
       medicationCount != lastDisplayedMedicationCount)) {
    displayBoxStatus();
    return;
  }

  // Update time every second (partial refresh to avoid flicker)
  static unsigned long lastTimeUpdate = 0;
  if (currentDisplay == DISPLAY_BOX_STATUS &&
      millis() - lastTimeUpdate > 1000) {
    tft.fillRect(30, 10, 180, 35, TFT_BLACK);
    tft.setTextSize(4);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(30, 10);
    tft.print(rtcGetTime());
    lastTimeUpdate = millis();
  }
}

// ==================== End of Box Status Display Module ====================

void updateLCDStatus() {
  // Update status bar every second (non-blocking)
  unsigned long now = millis();
  if (now - lastLCDUpdate >= 1000) {
    lastLCDUpdate = now;

    // Only update if not showing medication reminder
    // (Check if any medication is triggered)
    bool anyTriggered = false;
    for (int i = 0; i < medicationCount; i++) {
      for (int j = 0; j < medications[i].schedule_times_count; j++) {
        if (medications[i].triggered[j]) {
          anyTriggered = true;
          break;
        }
      }
      if (anyTriggered) break;
    }

    // Only update time display if showing main screen
    if (!anyTriggered && medicationCount >= 0) {
      // Clear bottom status area
      tft.fillRect(0, 200, 320, 40, TFT_BLACK);

      // Display current time
      tft.setTextSize(2);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(40, 205);
      tft.print("Time: ");
      tft.print(rtcGetTime());
    }
  }
}
