#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "Sensor.h"

class Thermistor : public Sensor {
public:
  virtual void init();
  virtual void read();

  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);
  
  float reading;
};

#endif // THERMISTOR_H
