#ifndef RTC_H
#define RTC_H

#include <Arduino.h>
#include "RTClib.h"

void rtcBegin();
String rtcGetTime();
bool rtcSetTime(const String& datetime);

#endif