#ifndef BUOYANCY_H
#define BUOYANCY_H
#include "Sensor.h"
#include "elapsedMillis.h"

typedef enum {
  DISABLED=0,
  ARMED=1,
  ACTIVE=2,
  RETURN=3
} state_t;


class Buoyancy : public Sensor {
public:
  Buoyancy() {
    strcpy(name,"buoyancy");
    enabled=false;
    state=DISABLED;
    T_deriv=0.0; // disabled while testing
    T_lowpass=2.0;
    deadband=0.05;
    depth_target=0.1;
    mission_seconds=30;
  }
  virtual void init();
  virtual void async_read();
  void display(void);
  void start_mission(void);

  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);
  virtual void enter_sample_loop(void);
  virtual void exit_sample_loop(void);
  
  void disable(void);
  void enable(void);

  state_t state;
  
  // atmospheric pressure at time of deployment
  int32_t atm_press_dPa;

  // control system parameters
  float T_deriv;
  float T_lowpass;
  float deadband;

  // mission parameters
  int mission_seconds;
  float depth_target; // positive down, meters

  // mission/control state
  elapsedMillis mission_time;
  // may eventually optimize with int32, but start with floats for easier dev.
  float w_mps; // vertical velocity, positive up, m/sec
  elapsedMillis since_last; // time since last pressure measurement
  int32_t pressure_last_dPa; // last pressure observation
};
  
#endif // BUOYANCY_H
