#ifndef BUOYANCY_H
#define BUOYANCY_H
#include "Sensor.h"

// which motor output is being used
#define MTR_SEL MOTOR_A
#define MTR_POS MOTOR_FWD; // check!!!!
#define MTR_NEG MOTOR_REV; // !
  

class Buoyancy : public Sensor {
public:
  Buoyancy() {
    strcpy(name,"buoyancy");
    enabled=false;
    
  }
  virtual void init();
  virtual void async_read();
  void display(void);

  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);

  void disable(void);
  void enable(void);

  // atmospheric pressure at time of deployment
  float atm_press;

  float T_deriv;
  float T_lowpass;
  float deadband;

  int mission_seconds;
  float depth_target;
  
private:
  // adc readings when motors are off
};
  
#endif // BUOYANCY_H
