#include "cfg_seaduck.h"

#ifdef HAS_RTC_DS3231

#include "serialmux.h"

#include <TimeLib.h>

#include "SeaDuck.h"
#include "rtc_ds3231.h"

#include <AWire.h>

#include "DS3231.h"

void print_date(DateTime &dt);
static byte conv_digits(const char* p,int digits);

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


// stolen from RTClib.cpp
static byte conv_digits(const char* p,int digits)
{
  uint16_t v = 0;
  for(int digit=0;digit<digits;digit++) {
    v*=10;
    if ('0' <= *p && *p <= '9') 
      v += *p - '0';
    p++;
  }
  return v;
}

void RTC_DS3231::set_datetime(const char *str) {
  // format is YYYY-MM-DDTHH:MM:SS
  // where the date/time separator is arbitrary (T, space, _, etc.)
  if( strlen(str) != strlen("YYYY-MM-DD HH:MM:SS") ) {
    mySerial.println("Error: format is YYYY-MM-DD HH:MM:SS");
    return;
  }

  ds3231.setYear(conv_digits(str+2,2)); // two digit year
  ds3231.setMonth(conv_digits(str+5,2));
  ds3231.setDate(conv_digits(str+8,2));
  ds3231.setHour(conv_digits(str+11,2));
  ds3231.setMinute(conv_digits(str+14,2));
  ds3231.setSecond(conv_digits(str+17,2));
}

bool RTC_DS3231::dispatch_command(const char *cmd, const char *cmd_arg) {
  if ( strcmp(cmd,"rtc_status")==0 ) {
    status();
  } else if ( strcmp(cmd,"rtc_datetime")==0 ) {
    if(cmd_arg) {
      set_datetime(cmd_arg);
    } else {
      status();
    }
  } else if ( strcmp(cmd,"rtc_watch")==0 ) {
    watch();
  } else if ( strcmp(cmd,"rtc_enable")==0 ) {
    if(cmd_arg) {
      enabled=(bool)atoi(cmd_arg);
    } else {
      mySerial.print("rtc_enable="); mySerial.println( enabled );
    }
  } else {
    return false;
  }
  return true;
}

void RTC_DS3231::help(void) {
  mySerial.println("  Real time clock");
  mySerial.println("    rtc_status       # print current time in seconds");
  mySerial.println("    rtc_watch        # check incrementing of RTC");
  mySerial.println("    rtc_datetime=YYYY-MM-DD HH:MM:SS # set clock");
  mySerial.println("    rtc_enable[=0,1] # enable/disable ");
  
}

void printDigits(int digits) {
 // utility function for digital clock display: prints preceding colon and leading 0
 if (digits < 10)
   mySerial.print('0');
 mySerial.print(digits);
}

void RTC_DS3231::status() {
  reading_seconds=0;

  read();

  mySerial.print("seconds=");
  mySerial.println(reading_seconds);

  DateTime now(reading_seconds); 
  mySerial.print("datetime=");
  print_date(now);
}

void print_date(DateTime &dt) {
  mySerial.print(dt.year());
  mySerial.print("-");
  printDigits(dt.month());
  mySerial.print("-");
  printDigits(dt.day());

  mySerial.print(" ");
  printDigits(dt.hour());
  mySerial.print(":");
  printDigits(dt.minute());
  mySerial.print(":");
  printDigits(dt.second());
  mySerial.println();
}

void RTC_DS3231::watch() {
  mySerial.println("Looping 100+ millis:");
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
