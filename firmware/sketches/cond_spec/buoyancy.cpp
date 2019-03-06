#include "cfg_seaduck.h"
#ifdef HAS_BUOYANCY

#ifndef HAS_PRESSURE
#error BUOYANCY relies on PRESSURE
#endif

#ifndef HAS_MOTOR
#error BUOYANCY relies on MOTOR
#endif

/* 
 Buoyancy interface:
 
 Command file should have something like:

 buoy_duration=600 # will go to target depth for this long
 buoy_depth=2.0 # aim for 2.0m depth

 Controller parameters:
   buoy_T_deriv=10.0 # time scale for derivative term in seconds
   buoy_T_lowpass=1.0 # time scale in seconds for low pass of pressure to come up with
      vertical velocity.
   buoy_deadband=0.5 # when setpoint is this close to actual, don't do anything.

 buoy_start # start running the buoyancy controller
 
 Operation:
   If BT works:
    0. Command plunger full out, max buoyancy.
    1. Tune weights to be just at the surface (in a bucket)
    2. Start mission
    3. after the mission_seconds, it goes full positive, waits.
    4. Reconnect bluetooth, download.

   If no BT:
    Probably have to flip it sideways to signal a start, or put it on a timer.
    0. Resting position is plunger full out, max buoyancy
    1. Tune weights to be just at the surface
    2. Flip horizontal for 5 seconds to signal start of mission
    3. It runs the mission, goes full positive, waits.

   I'm thinking we have to use the compass here -- there are too many possibilities
   of false triggering if just using the accelerometer.

 Not yet
 on_invert  # how to interpret being inverted for 3 seconds 
 on_invert=restart_depth # cycle to start of depth schedule
 on_invert=none # do nothing

 */


#include "motor.h"
#include "buoyancy.h"

#include "pressure.h"
extern Pressure pressure;

#include "motor.h"
extern Motor motor;

void Buoyancy::init() {
}

void Buoyancy::disable(void){
  enabled=false;
}

void Buoyancy::enable(void) {
  enabled=true;
}

void Buoyancy::async_read() {
  // For now this just updates readings
  
  
  // and keep the ball in the air.
  pop_fn_and_call();
}

void Buoyancy::display(void) {
  Serial.print("buoyancy_enable="); Serial.println( enabled );
}

void set_float(char *arg,float *orig,char *label) {
  float tmp;
  char *endp;
  
  if (arg==NULL) {
    Serial.print(label); Serial.print("="); Serial.println(*orig);
    return;
  } else {
    tmp=(float)strtod(arg,&endp);
    
    if ( endp==arg ) {
      Serial.println("Could not parse number");
      return;
    } else {
      *orig=tmp;
    }
  }
}


void Buoyancy::start_mission(void) {
  // enable motor
  // go full positive
  // record start time
  //   - is it worthwhile to have an independent timer for coming back to the surface?
  // initialize PD state

  // Need to be sure that pressure is being sampled regularly.
  // Easiest thing is to tie the critical action code to the sample loop.
  // And maybe have code that if we exit the sample loop it forces full positive.

  motor.enable();

  motor.command(MTR_SEL,MTR_POS);

  // be sure it goes until it hits the limit.
  while ( motor.wait_and_stop() != MOTOR_LIMIT ) {
    Serial.println("Appears not have hit its limit");
  }

  pick up HERE
}

void set_int(char *arg,int *orig,char *label) {
  int tmp;
  char *endp;
  
  if (arg==NULL) {
    Serial.print(label); Serial.print("="); Serial.println(*orig);
    return;
  } else {
    tmp=strtoi(arg,&endp);
    
    if ( endp==arg ) {
      Serial.println("Could not parse number");
      return;
    } else {
      *orig=tmp;
    }
  }
}


bool Buoyancy::dispatch_command(const char *cmd, const char *cmd_arg) {
  float tmp;
  if ( !strcmp(cmd,"buoy") ) {
    display();
  } else if(!strcmp(cmd,"buoy_T_deriv")) {
    set_float(cmd_arg,&T_deriv,"buoy_T_deriv");
  } else if(!strcmp(cmd,"buoy_T_lowpass")) {
    set_float(cmd_arg,&T_lowpass,"buoy_T_lowpass");
  } else if(!strcmp(cmd,"buoy_deadband")) {
    set_float(cmd_arg,&deadband,"buoy_deadband");
  } else if(!strcmp(cmd,"buoy_duration")) {
    set_int(cmd_arg,&mission_seconds,"buoy_duration");
  } else if(!strcmp(cmd,"buoy_depth")) {
    set_float(cmd_arg,&mission_seconds,"buoy_depth");
  } else if(!strcmp(cmd,"buoy_start")) {
    start_mission();
  } else {
    return false;
  }
  return true;
}


void Buoyancy::help() {
  Serial.println("  Buoyancy");
  Serial.println("    buoy                     # report status");
  Serial.println("    buoy_T_deriv[=seconds]   # time scale for derivative term");
  Serial.println("    buoy_T_lowpass[=seconds] # time scale for depth lowpass");
  Serial.println("    buoy_deadband[=meters]   # error deadband");
  Serial.println("    buoy_duration[=seconds]  # duration for target depth");
  Serial.println("    buoy_depth[=meters]      # target depth");
}
  
void Buoyancy::write_frame_info(Print &out) { }
void Buoyancy::write_data(Print &out){ }

#endif // HAS_BUOYANCY
