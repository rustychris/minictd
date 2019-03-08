#include "cfg_seaduck.h"
#ifdef HAS_RTC_TEENSY

#include <TimeLib.h>

#include "SeaDuck.h"
#include "rtclock.h"

#include <core_pins.h> // needed for Teensy3Clock

void RTClock::init() {
  ;
}

void RTClock::async_read() {
  reading_seconds = Teensy3Clock.get();
  reading_partial = RTC_TPR;
  pop_fn_and_call();
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
  mySerial.println("  Real time clock");
  mySerial.println("    rtc_status # print current time in seconds");
  mySerial.println("    rtc_watch  # check incrementing of RTC");
}

void printDigits(int digits) {
 // utility function for digital clock display: prints preceding colon and leading 0
 if (digits < 10)
   mySerial.print('0');
 mySerial.print(digits);
}

void RTClock::status() {
  read();
  mySerial.print(reading_seconds);
  mySerial.print(" ");
  mySerial.println(reading_partial);
  // Print out a nicer version of that:
  mySerial.print(year(reading_seconds));
  mySerial.print("-");
  mySerial.print(month(reading_seconds));
  mySerial.print("-");
  mySerial.print(day(reading_seconds));

  mySerial.print(" ");
  printDigits(hour(reading_seconds));
  mySerial.print(":");
  printDigits(minute(reading_seconds));
  mySerial.print(":");
  printDigits(second(reading_seconds));
  mySerial.println();
}

void RTClock::watch() {
  mySerial.println("Looping 100+ millis:");
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

#endif // HAS_RTC_TEENSY
