#include "RTC.h"

RTC_DS3231 rtc;

char daysOfTheWeek[7][12] = {
  "Sunday", "Monday", "Tuesday", "Wednesday",
  "Thursday", "Friday", "Saturday"
};


void rtcBegin() {
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting to compile time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

String rtcGetTime() {
  DateTime now = rtc.now();

  String yearStr = String(now.year(), DEC);
  String monthStr = (now.month() < 10 ? "0" : "") + String(now.month(), DEC);
  String dayStr = (now.day() < 10 ? "0" : "") + String(now.day(), DEC);
  String hourStr = (now.hour() < 10 ? "0" : "") + String(now.hour(), DEC);
  String minuteStr = (now.minute() < 10 ? "0" : "") + String(now.minute(), DEC);
  String secondStr = (now.second() < 10 ? "0" : "") + String(now.second(), DEC);
  String dayOfWeek = daysOfTheWeek[now.dayOfTheWeek()];

  String formatted = hourStr + ":" + minuteStr;

  return formatted;
}

bool rtcSetTime(const String& datetime) {
  // Expected format: "2025-10-20 14:30:00"
  // Parse the datetime string
  int year, month, day, hour, minute, second;

  int result = sscanf(datetime.c_str(), "%d-%d-%d %d:%d:%d",
                     &year, &month, &day, &hour, &minute, &second);

  if (result != 6) {
    Serial.println("Error: Invalid datetime format. Expected YYYY-MM-DD HH:MM:SS");
    return false;
  }

  // Validate ranges
  if (year < 2000 || year > 2099 || month < 1 || month > 12 ||
      day < 1 || day > 31 || hour < 0 || hour > 23 ||
      minute < 0 || minute > 59 || second < 0 || second > 59) {
    Serial.println("Error: Datetime values out of range");
    return false;
  }

  // Set RTC time
  rtc.adjust(DateTime(year, month, day, hour, minute, second));

  Serial.print("RTC time set to: ");
  Serial.println(datetime);

  return true;
}

