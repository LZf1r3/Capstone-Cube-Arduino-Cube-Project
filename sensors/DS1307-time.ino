#include <Wire.h>
#include "RTClib.h"

RTC_DS1307 rtc;

// ⭐ CHANGE THIS WHEN NEEDED ⭐
#define AUTO_SET_TIME true
// true  = set RTC to computer time on upload
// false = keep existing RTC time

char timeBuffer[32];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  Serial.println("Starting DS1307 RTC...");

  if (!rtc.begin()) {
    Serial.println("❌ RTC not found. Check wiring.");
    while (1);
  }

  if (!rtc.isrunning()) {
    Serial.println("⚠ RTC not running (battery missing/dead).");
  }

  if (AUTO_SET_TIME) {
    Serial.println("⏱ Setting RTC to computer time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("✅ RTC updated.");
  } else {
    Serial.println("✔ RTC keeping stored time.");
  }

  Serial.println();
}

void loop() {
  DateTime now = rtc.now();

  // Format nicely
  snprintf(timeBuffer, sizeof(timeBuffer),
           "%04d-%02d-%02d  %02d:%02d:%02d",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());

  Serial.print("RTC Time: ");
  Serial.println(timeBuffer);

  delay(500);
}