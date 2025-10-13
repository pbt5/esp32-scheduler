#include "RTC.h"
#include "Wire.h"
#include <ArduinoJson.h>
#include <WiFi.h>

#define LED_PIN 13
#define SDA_PIN 22
#define SCL_PIN 20
#define DETECT_PIN 12

StaticJsonDocument<512> scheduleDoc;


void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DETECT_PIN, INPUT_PULLUP);

  // Setup I2c
  Wire.begin(SDA_PIN, SCL_PIN); // TODO: Fix pins (double check pin numbers for different esp32s)

  // Setup clock
  rtcBegin();

  // Set schedule
  const char* medicineJson = R"rawliteral(
  {
    "1": {"time":"05:00", "duration":60, "dose":1, "note":"Advil: Take with food."},
    "2": {"time":"21:00", "duration":30, "dose":2, "note":"Tylenol: Eat before taking!"},
    "3": {"time":"11:30", "duration":60, "dose":3, "note":"Allergy pills"}
  }
  )rawliteral";

  setSchedule(medicineJson);
}

void loop() {
  blink();
  setLedIfTime();

  // Serial.println(rtcGetTime());

  // int read = digitalRead(DETECT_PIN);
  // Serial.println(read);

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

struct BoxLED {
  int boxNumber;
  int pin;
};

BoxLED dictLED[] = {
  {1, 27},
  {2, 33},
  {3, 15},
  {4, 32},
  {5, 14}
};

struct Schedule {
  String time;
  int dose;
  int duration;
  String note;
  int ledPin;
  bool triggered;
  unsigned long triggerMillis;
};

Schedule schedules[5];
int scheduleCount = 0;

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
    schedules[scheduleCount].ledPin = dictLED[scheduleCount].pin;
    schedules[scheduleCount].triggered = false;
    schedules[scheduleCount].triggerMillis = 0;

    pinMode(schedules[scheduleCount].ledPin, OUTPUT);
    digitalWrite(schedules[scheduleCount].ledPin, LOW);

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

  for (int i = 0; i < scheduleCount; i++) {
    if (!schedules[i].triggered && currentTime == schedules[i].time) {
      digitalWrite(schedules[i].ledPin, HIGH);
      schedules[i].triggered = true;
      schedules[i].triggerMillis = now;

      // TODO: Display code
    }

    if (schedules[i].triggered && now - schedules[i].triggerMillis >= (unsigned long)schedules[i].duration * 60000) {
      digitalWrite(schedules[i].ledPin, LOW);
      schedules[i].triggered = false;
    }
  }
}

void detectOpenClose() {
  int read = digitalRead(DETECT_PIN);
  if (read == 1) {
    Serial.println("Open");
    // TODO: Send the data to the computer
  } else {
    Serial.println("Close");
    // TODO: Send the data to computer
  }
}

// Non-blocking on-board LED for debugging purposes
// Could be used as a power signal for the box
#define LED_PIN LED_BUILTIN  
unsigned long blinkMillis = 0;
const long interval = 1000;
void blink() {
  unsigned long currentMillis = millis();

  if (currentMillis - blinkMillis >= interval) {
    blinkMillis = currentMillis;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}