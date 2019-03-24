#ifndef MOTOR_H
#define MOTOR_H
#include "Sensor.h"
#include "elapsedMillis.h"

enum { 
  DISP_ENABLE=1,
  DISP_SENSE=2,
  DISP_CURRENT=4,
  DISP_POSITION=8,
  DISP_OFFSET=16,
  DISP_ALL=31,
  DISP_OFF=32 // show stats for motors that are off
};

enum {
  MOTOR_OFF=0,
  MOTOR_FWD=1,
  MOTOR_REV=2,
  MOTOR_ON=3, // FWD|REV
  MOTOR_LIMIT=4, // can be combined with FWD or REV
};

enum {
  MOTOR_A=1,
  MOTOR_B=2
};
  
class Motor : public Sensor {
public:
  Motor() {
    strcpy(name,"motor");
    enabled=false;
    a_status=MOTOR_OFF;
    b_status=MOTOR_OFF;
    // had been 0.015 with a decay of 0.08
    // but got an occasional false positive that way.
    a_current_threshold=0.010;
    b_current_threshold=0.010;
    a_decay=0.9;
    b_decay=0.9;
  }
  
  virtual void init();
  virtual void async_read();
  void display(void) { display(DISP_ALL|DISP_OFF); }
  void display(unsigned int select);

  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);

  void all_off(void);
  void disable(void);
  void enable(void);
  // returns 0 for stop on user input, and MOTOR_LIMIT if a limit was detected
  int wait_and_stop(void);

  void command(int motor,int cmd);
  void read_current(int motor);
  void update_position(void);
  
  elapsedMillis position_elapsed;

  // estimate of cumulative time in FWD in ms
  long a_position;
  long b_position;

  long position(int motor) { return (motor==MOTOR_A)?a_position:b_position; }
  void position_reset(int motor) {
    if(motor&MOTOR_A) a_position=0;
    if(motor&MOTOR_B) b_position=0;
  }

  int16_t status(int motor) { return (motor==MOTOR_A)?a_status:b_status; }
  
private:
  // if true, the motor driver is on.
  // if false, motor driver is disabled, and no drive signals
  // will be relayed to motor output.
  bool enabled;

  // adc readings when motors are off
  int a_sense_offset;
  int b_sense_offset;
  int a_sense;
  int b_sense;
  float a_current;
  float b_current;

  // repeated motor current readings below this threshold
  // are interpreted as a limit switch engaged.
  float a_current_threshold;
  float b_current_threshold;
  // janky, but there is some noise in the current readings, so this
  // averages it out.  a_current is updated with
  // a_current=a_decay*a_current + (1-a_decay)*<new current reading>
  float a_decay;
  float b_decay;
  
  int16_t a_status;
  int16_t b_status;
};
  
#endif // MOTOR_H
