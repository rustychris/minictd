#ifndef MOTOR_H
#define MOTOR_H
#include "Sensor.h"

class Motor : public Sensor {
public:
  Motor() {
    strcpy(name,"motor");
  }
  virtual void init();
  virtual void async_read();
  void display();

  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);

  void all_off(void);
  void disable(void);
  void enable(void);

private:
  // if true, the motor driver is on.
  // if false, motor driver is disabled, and no drive signals
  // will be relayed to motor output.
  bool enabled;

  // adc readings when motors are off
  int a_sense_offset;
  int b_sense_offset;
};
  
#endif // MOTOR_H
