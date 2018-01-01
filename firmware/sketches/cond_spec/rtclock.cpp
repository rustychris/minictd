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

void printDigits(int digits) {
 // utility function for digital clock display: prints preceding colon and leading 0
 if (digits < 10)
   Serial.print('0');
 Serial.print(digits);
}

void RTClock::status() {
  read();
  Serial.print(reading_seconds);
  Serial.print(" ");
  Serial.println(reading_partial);
  // Print out a nicer version of that:
  Serial.print(year(reading_seconds));
  Serial.print("-");
  Serial.print(month(reading_seconds));
  Serial.print("-");
  Serial.print(day(reading_seconds));

  Serial.print(" ");
  printDigits(hour(reading_seconds));
  Serial.print(":");
  printDigits(minute(reading_seconds));
  Serial.print(":");
  printDigits(second(reading_seconds));
  Serial.println();
}

void RTClock::watch() {
  Serial.println("Looping 100+ millis:");
  for(int i=0;i<30;i++) {
    status();
    delay(100);
  }
}

void RTClock::write_frame_info(Print &out) {
  out.print("('seconds','<i4'),('partial','<i4'),");
}

void RTClock::write_data(Print &out) {
  write_base16(out,(uint8_t*)(&reading_seconds),sizeof(reading_seconds));
  write_base16(out,(uint8_t*)(&reading_partial),sizeof(reading_partial));
}
