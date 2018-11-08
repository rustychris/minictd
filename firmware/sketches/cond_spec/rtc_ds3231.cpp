#include "cfg_seaduck.h"

#ifdef HAS_RTC_DS3231

#include <TimeLib.h>

#include "SeaDuck.h"
#include "rtc_ds3231.h"

#include <AWire.h>

#include "DS3231.h"

void print_date(DateTime &dt);

DS3231 ds3231;

void RTC_DS3231::init(void) {
  ;
}

void RTC_DS3231::async_read(void) {
  AWire.beginTransmission(CLOCK_ADDRESS);
  AWire.write(0);	// This is the first register address (Seconds)
  // We'll read from here on for 7 bytes: secs reg, minutes reg, hours, days, months and years.
  push_fn(this,(SensorFn)&RTC_DS3231::async_read2);
  AWire.onTransmitDone(pop_fn_and_call);
  AWire.sendTransmission();
}

void RTC_DS3231::async_read2(void) {
  AWire.onTransmitDone(NULL);
  push_fn(this,(SensorFn)&RTC_DS3231::async_read3);
  AWire.onReqFromDone(pop_fn_and_call);
  AWire.sendRequest(CLOCK_ADDRESS, 7);
}
void RTC_DS3231::async_read3(void) {
  AWire.onReqFromDone(NULL);
  uint8_t ss = bcd2bin(AWire.read() & 0x7F);
  uint8_t mm = bcd2bin(AWire.read());
  uint8_t hh = bcd2bin(AWire.read());
  AWire.read();
  uint8_t d = bcd2bin(AWire.read());
  uint8_t m = bcd2bin(AWire.read());
  uint16_t y = bcd2bin(AWire.read()) + 2000;

  DateTime dt(y, m, d, hh, mm, ss);
  reading_seconds=dt.unixtime();

  pop_fn_and_call();
}

bool RTC_DS3231::dispatch_command(const char *cmd, const char *cmd_arg) {
  if ( strcmp(cmd,"rtc_status")==0 ) {
    status();
  } else if ( strcmp(cmd,"rtc_watch")==0 ) {
    watch();
  } else if ( strcmp(cmd,"rtc_enable")==0 ) {
    if(cmd_arg) {
      enabled=(bool)atoi(cmd_arg);
    } else {
      Serial.print("rtc_enable="); Serial.println( enabled );
    }
  } else {
    return false;
  }
  return true;
}

void RTC_DS3231::help(void) {
  Serial.println("  Real time clock");
  Serial.println("    rtc_status       # print current time in seconds");
  Serial.println("    rtc_watch        # check incrementing of RTC");
  Serial.println("    rtc_enable[=0,1] # enable/disable ");
  
}

void printDigits(int digits) {
 // utility function for digital clock display: prints preceding colon and leading 0
 if (digits < 10)
   Serial.print('0');
 Serial.print(digits);
}

void RTC_DS3231::status() {
  reading_seconds=0;

  read();

  Serial.print("seconds=");
  Serial.println(reading_seconds);

  DateTime now(reading_seconds); 
  Serial.print("datetime=");
  print_date(now);
}

void print_date(DateTime &dt) {
  Serial.print(dt.year());
  Serial.print("-");
  printDigits(dt.month());
  Serial.print("-");
  printDigits(dt.day());

  Serial.print(" ");
  printDigits(dt.hour());
  Serial.print(":");
  printDigits(dt.minute());
  Serial.print(":");
  printDigits(dt.second());
  Serial.println();
}

void RTC_DS3231::watch() {
  Serial.println("Looping 100+ millis:");
  for(int i=0;i<30;i++) {
    status();
    delay(100);
  }
}

void RTC_DS3231::write_frame_info(Print &out) {
  out.print("('seconds','<i4'),");
}

void RTC_DS3231::write_data(Print &out) {
  write_base16(out,(uint8_t*)(&reading_seconds),sizeof(reading_seconds));
}

#endif // HAS_RTC_DS3231
