#ifndef MOTOR_H
#define MOTOR_H
#include "Sensor.h"

enum { 
  DISP_ENABLE=1,
  DISP_SENSE=2,
  DISP_CURRENT=4,
  DISP_ALL=7
};


class Motor : public Sensor {
public:
  Motor() {
    strcpy(name,"motor");
    enabled=false;
  }
  virtual void init();
  virtual void async_read();
  void display(void) { display(DISP_ALL); }
  void display(unsigned int select);

  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);

  void all_off(void);
  void disable(void);
  void enable(void);
  void wait_and_stop(void);
  
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
