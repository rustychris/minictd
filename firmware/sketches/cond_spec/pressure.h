#ifndef PRESSURE_H
#define PRESSURE_H

#include "Sensor.h"

class Pressure : public Sensor {
public:
  virtual void init();
  virtual void read();
  void display();
  
  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);
  
  int32_t pressure_baseline_dPa;
  
  int32_t pressure_abs_dPa;
  int32_t temperature_c100;
};

#endif // PRESSURE_H
