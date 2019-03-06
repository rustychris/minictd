#ifndef BUOYANCY_H
#define BUOYANCY_H
#include "Sensor.h"
#include "elapsedMillis.h"


class Buoyancy : public Sensor {
public:
  Buoyancy() {
    strcpy(name,"buoyancy");
    enabled=false;
    
  }
  virtual void init();
  virtual void async_read();
  void display(void);
  void start_mission(void);

  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);

  void disable(void);
  void enable(void);

  // atmospheric pressure at time of deployment
  int32_t atm_press_dPa;

  // control system parameters
  float T_deriv;
  float T_lowpass;
  float deadband;

  // mission parameters
  int mission_seconds;
  float depth_target;

  // mission/control state
  elapsedMillis mission_time;
  // may eventually optimize with int32, but start with floats for easier dev.
  float w_mps; // vertical velocity, positive up, m/sec
  elapsedMillis since_last; // time since last pressure measurement
  int32_t pressure_last_dPa; // last pressure observation
};
  
#endif // BUOYANCY_H
