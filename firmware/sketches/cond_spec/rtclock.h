#ifndef RTCLOCK_H
#define RTCLOCK_H

#include "Sensor.h"

class RTClock : public Sensor {
public:
  virtual void init();
  virtual void async_read();

  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);
  
  // unsigned long reading_seconds;
  time_t reading_seconds;
  unsigned int reading_partial;

  void status();
  void watch();
};

#endif // RTCLOCK_H
