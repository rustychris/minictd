#ifndef RTC_DS3231_H
#define RTC_DS3231_H

#ifdef HAS_RTC_DS3231

#include "Sensor.h"

class RTC_DS3231 : public Sensor {
public:
  RTC_DS3231(void) {
    strcpy(name,"rtc_ds3231");
  }
  
  virtual void init(void);
  virtual void async_read(void);
  void async_read2(void);
  void async_read3(void);

  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help(void);
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);

  // endpoint for async_read
  volatile uint32_t reading_seconds;

  void status(void);
  void watch(void);
};

#endif // HAS_RTC_DS3231
#endif // RTC_DS3231_H
