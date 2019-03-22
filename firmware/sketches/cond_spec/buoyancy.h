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
  // estimate of water volume in piston in ml
  float piston_water_volume(void);

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

  // PID[A] controller
  //   parameters
  float T_deriv;// timescales for deriv and accel terms.
  float G_prop, R_integ; // R as in rate, as in inverse time
  float transit_fraction;

  float T_lowpass;
  float deadband;

  // pressure lowpass: 2nd order butterworth
  float lpA[3];
  float lpB[3];
  float lpx[3];
  float lpy[3];
  
  // calculated terms
  float ctrl_prop; 
  float ctrl_deriv;
  float ctrl_integ; // TODO: be sure this is properly initialized

  float ctrl; // sum of the control terms
  
  // lowpass the velocity
  void update_z(void);
  
  // mission parameters
  int mission_seconds;
  float depth_target; // positive down, meters

  // mission/control state
  elapsedMillis mission_time;
  // may eventually optimize with int32, but start with floats for easier dev.
  float w_mps; // vertical velocity, positive up, m/sec
  elapsedMillis since_last; // time since last pressure measurement
};
  
#endif // BUOYANCY_H
