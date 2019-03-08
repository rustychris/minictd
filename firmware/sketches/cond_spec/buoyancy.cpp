#include "cfg_seaduck.h"
#ifdef HAS_BUOYANCY

#ifndef HAS_PRESSURE
#error BUOYANCY relies on PRESSURE
#endif

#ifndef HAS_MOTOR
#error BUOYANCY relies on MOTOR
#endif

#include "serialmux.h"

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


const char *state_labels[]={"DISABLED",
                            "ARMED",
                            "ACTIVE",
                            "RETURN"};

void Buoyancy::init() {
}

void Buoyancy::disable(void){
  enabled=false;
}

void Buoyancy::enable(void) {
  enabled=true;
}

void Buoyancy::enter_sample_loop(void) {
  // arguably could set baseline atmospheric pressure here, too
  mission_time=0;
}

void Buoyancy::exit_sample_loop(void) {
  switch(state) {
  case DISABLED:
    break;
  case ARMED:
    state=DISABLED; // could leave it...
    break;
  case ACTIVE:
    state=RETURN;
    motor.command(BUOY_MTR_SEL,BUOY_MTR_POS);
    break;
  case RETURN:
    break;
  }
}

void Buoyancy::async_read() {
  if ( state==DISABLED ) {
    pop_fn_and_call();
    return;
  }

  if (state==ARMED) {
    state=ACTIVE;
  }

  // check for end of mission
  if( (state==ACTIVE) && (mission_time>1000*mission_seconds) ) {
    mySerial.println("# Buoy end of mission - full positive");
    state=RETURN;
  }
  if(state==RETURN) {    
    motor.command(BUOY_MTR_SEL,BUOY_MTR_POS);
    pop_fn_and_call();
    return;
  }

  // Otherwise we're in the control loop
  
  // Update w estimatte
  float w_instant;

  // from pressure.cpp --
  //  mbar is dPa/10
  //  bar is 100 kPa, so mbar 100Pa
  //  and dPa would then be 10Pa, so decaPascals.
  
  // so the raw readings are potentially mm of water.
  // assuming 'actual' readings are in decaPascals,
  // scale that by 1000, and 
  
  // 1000*dPa / ms, approx mm/s
  // May need further scaling to get the right resolution.

  // mm/ms ~ m/s
  long dt_ms=since_last;
  since_last=0;
  
  w_instant=(pressure_last_dPa - pressure.pressure_abs_dPa);
  w_instant /= dt_ms;

  // T_lowpass convert to ms, 
  float alpha=dt_ms/(1000*T_lowpass);

  if (alpha<1.0) {
    w_mps=alpha*w_instant + (1-alpha)*w_mps;
  } else {
    w_mps=w_instant;
  }

  pressure_last_dPa=pressure.pressure_abs_dPa;

  // CONTROL
  // Current depth, positive up
  float z_m=(atm_press_dPa-pressure_last_dPa)/1000.0;
  float prop=z_m+depth_target; // depth_target is positive down
  //   depth error   +   derivative term
  float PD=prop + w_mps*T_deriv;

  mySerial.print("# w_instant=");
  mySerial.print(w_instant,4);
  mySerial.print("  w_mps=");
  mySerial.print(w_mps,6);
  mySerial.print("  z=");
  mySerial.print(z_m,4);

  if(PD>deadband) { // above target
    mySerial.print("  NEG");
    motor.command(BUOY_MTR_SEL,BUOY_MTR_NEG);
  } else if(PD<deadband) { // below target
    mySerial.print("  POS");
    motor.command(BUOY_MTR_SEL,BUOY_MTR_POS);
  } else {
    mySerial.print("  OFF");
    motor.command(BUOY_MTR_SEL,MOTOR_OFF);
  }

  mySerial.println();
  
  // and keep the ball in the air.
  pop_fn_and_call();
}

void Buoyancy::display(void) {
  mySerial.print("buoyancy_enable="); mySerial.println(enabled);
  mySerial.print("buoyancy_state="); mySerial.println(state_labels[state]);
  mySerial.print("buoy_T_deriv="); mySerial.println(T_deriv);
  mySerial.print("buoy_T_lowpass="); mySerial.println(T_lowpass);
  mySerial.print("buoy_deadband="); mySerial.println(deadband);
  mySerial.print("buoy_depth="); mySerial.println(depth_target);
  mySerial.print("buoy_duration="); mySerial.println(mission_seconds);
  mySerial.print("buoy_atm_press_dPa="); mySerial.println(atm_press_dPa);
  mySerial.print("buoy_w="); mySerial.println(w_mps);
  mySerial.print("buoy_mission_time="); mySerial.println(mission_time);
}

void set_float(const char *arg,float *orig,const char *label) {
  float tmp;
  char *endp;
  
  if (arg==NULL) {
    mySerial.print(label); mySerial.print("="); mySerial.println(*orig);
    return;
  } else {
    tmp=(float)strtod(arg,&endp);
    
    if ( endp==arg ) {
      mySerial.println("Could not parse number");
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

  motor.command(BUOY_MTR_SEL,BUOY_MTR_POS);

  // be sure it goes until it hits the limit.
  while ( motor.wait_and_stop() != MOTOR_LIMIT ) {
    mySerial.println("Appears not have hit its limit");
    motor.command(BUOY_MTR_SEL,BUOY_MTR_POS);
  }

  mySerial.println("At full positive -- armed to start mission");

  // Make sure we have an up to date pressure reading
  pressure.read();
  atm_press_dPa=pressure.pressure_abs_dPa;
  mySerial.print("Atmospheric pressure recorded as ");
  mySerial.print(atm_press_dPa/1000.0f);
  mySerial.println("dbar");
  
  // Set Buoyancy state so that once the sample loop is going
  // the PD loop will begin.
  enabled=true; // affects whether sample loop will
  state=ARMED;
}

void set_int(const char *arg,int *orig,const char *label) {
  int tmp;
  char *endp;
  
  if (arg==NULL) {
    mySerial.print(label); mySerial.print("="); mySerial.println(*orig);
    return;
  } else {
    tmp=strtol(arg,&endp,10);
    
    if ( endp==arg ) {
      mySerial.println("Could not parse number");
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
    set_float(cmd_arg,&depth_target,"buoy_depth");
  } else if(!strcmp(cmd,"buoy_start")) {
    start_mission();
  } else {
    return false;
  }
  return true;
}


void Buoyancy::help() {
  mySerial.println("  Buoyancy");
  mySerial.println("    buoy                     # report status");
  mySerial.println("    buoy_T_deriv[=seconds]   # time scale for derivative term");
  mySerial.println("    buoy_T_lowpass[=seconds] # time scale for depth lowpass");
  mySerial.println("    buoy_deadband[=meters]   # error deadband");
  mySerial.println("    buoy_duration[=seconds]  # duration for target depth");
  mySerial.println("    buoy_depth[=meters]      # target depth");
  mySerial.println("    buoy_start               # prepare and start mission");
}
  
void Buoyancy::write_frame_info(Print &out) { }
void Buoyancy::write_data(Print &out){ }

#endif // HAS_BUOYANCY
