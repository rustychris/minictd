#include <core_pins.h> // needed for Teensy3Clock
#include <TimeLib.h>

#include "SeaDuck.h"
#include "rtclock.h"

void RTClock::init() {
  ;
}

void RTClock::read() {
  reading_seconds = Teensy3Clock.get();
  reading_partial = RTC_TPR;
}

bool RTClock::dispatch_command(const char *cmd, const char *cmd_arg) {
  if ( strcmp(cmd,"rtc_status")==0 ) {
    status();
  } else if ( strcmp(cmd,"rtc_watch")==0 ) {
    watch();
  } else {
    return false;
  }
  return true;
}

void RTClock::help() {
  Serial.println("  Real time clock");
  Serial.println("    rtc_status # print current time in seconds");
  Serial.println("    rtc_watch  # check incrementing of RTC");
}

void RTClock::status() {
  read();
  Serial.print(reading_seconds);
  Serial.print(" ");
  Serial.println(reading_partial);
}

void RTClock::watch() {
  Serial.println("Looping 100+ millis:");
  for(int i=0;i<30;i++) {
    status();
    delay(100);
  }
}
