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
  // set butterworth coefficients
  // calculated offline from this:
  // f_samp_Hz=(1./self.dt_control)
  // f_nyq_Hz=f_samp_Hz/2.0
  // f_cut_Hz=1./self.T_w_est
        
  // B,A = butter(2, # second order
  //              f_cut_Hz/f_nyq_Hz, # normalized [0,1] cutoff, relative to nyquist
  //             )
  // with dt_control=0.1 (s), T_w_est=10.0 (s)

  lpA[0]=1.0f;
  lpA[1]=-1.91119707f;
  lpA[2]=0.91497583f;
  lpB[0]=0.00094469f;
  lpB[1]=0.00188938f;
  lpB[2]=0.00094469f;
  lpx[0]=lpx[1]=lpx[2]=0.0f;
  lpy[0]=lpy[1]=lpy[2]=0.0f;

  // And some reasonable defaults for PID parameters
  G_prop=5;
  T_deriv=19;
  R_integ=0.12;
  transit_fraction=0.1;
  deadband=0.0; // come back to this one.
}

void Buoyancy::disable(void){
  enabled=false;
}

void Buoyancy::enable(void) {
  enabled=true;
}

void Buoyancy::update_z(void) {
  // dPa ~ mm, convert to m, positive up. 0 at surface
  float z_inst=(atm_press_dPa - pressure.pressure_abs_dPa)/1000.0f;

  // butterworth lowpass
  // B: numerator
  // A: denominator
  lpx[2]=lpx[1];
  lpx[1]=lpx[0];
  lpx[0]=z_inst;
  lpy[2]=lpy[1];
  lpy[1]=lpy[0];

  lpy[0]=1./lpA[0]*(lpB[0]*lpx[0] + lpB[1]*lpx[1] + lpB[2]*lpx[2]
                    -lpA[1]*lpy[1] - lpA[2]*lpy[2]);
  
  // w_est=(lpy[0]-lpy[1])/dt_control;
  // a_est=(lpy[0]-2*lpy[1]+lpy[2])/(dt_control*dt_control);
  // z_est=lpy[0];
  // z_instant=lpx[0];
}

void Buoyancy::enter_sample_loop(void) {
  // arguably could set baseline atmospheric pressure here, too
  mission_time=0;
  ctrl_integ=0.0f; // start with a clean slate.
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

  // Update the lowpass vertical estimates.
  update_z();

  // lpy[0] has the current lowpass vertical position, positive up.
  // lpy[1] and lpy[2] have the two previous estimates
  // these are all meters.

  long dt_ms=since_last;
  since_last=0;

  // m * ms/s / ms
  w_mps=(lpy[0]-lpy[1])*1000.0f/dt_ms;
  
  // CONTROL
  // Current depth, positive up
  float z_m=lpx[0];
  // instantaneous error used for proportional and integral values
  float err=z_m+depth_target; // depth_target is positive down

  ctrl_prop=G_prop*err;
  ctrl_integ+=err*dt_ms*R_integ/1000.f;
  ctrl_deriv=T_deriv*w_mps;

  // the requested piston water volume
  ctrl=ctrl_prop+ctrl_integ+ctrl_deriv;

  mySerial.print("  w_mps=");
  mySerial.print(w_mps,6);
  mySerial.print("  z=");
  mySerial.print(z_m,4);

  if ( -z_m/depth_target < transit_fraction ) {
    // back-calculate what the integral term would be to
    // keep going, so that when we submerge below
    // transit fraction it's like the integral term has
    // been wound-up just the right amount.  need a better
    // explanation than that.

    // on paper this should also subtract the ctrl_deriv,
    // but in simulation results were slightly better with adding
    // ctrl_deriv.
    //                              ms            ml/s           s/ms
    float piston_ml=motor.position(BUOY_MTR_SEL)*PISTON_RATE * 0.001f;
    ctrl_integ=piston_ml - ctrl_prop + ctrl_deriv;
    // DBG:
    Serial.print("ctrl_integ="); Serial.print(ctrl_integ,4);
    Serial.print("  piston_ml="); Serial.print(piston_ml,4);
    Serial.print("  ctrl_prop="); Serial.print(ctrl_prop,4);
    Serial.print("  ctrl_deriv="); Serial.print(ctrl_deriv,4);
    Serial.println();
    
    ctrl=BUOY_PISTON_ML; // max neg.
  } 

  // ctrl is target volume of water in piston in ml
  // convert to position in ms
  long position_request=(long)( ctrl*1000.f/PISTON_RATE );
  long change_request=position_request-motor.position(BUOY_MTR_SEL);

  if(change_request>deadband) {
    // above target depth, go more negative by increasing the
    // water in the piston
    // force a transition through OFF if motor is currently going in the wrong
    // direction.
    
    if(motor.status(BUOY_MTR_SEL) & BUOY_MTR_NEG) {
      // already going negative
      mySerial.print("  neg");
    } else if(motor.status(BUOY_MTR_SEL) & BUOY_MTR_POS) {
      // positive, so go through off.
      mySerial.print("  OFF...");
      motor.command(BUOY_MTR_SEL,MOTOR_OFF);
    } else {
      // off, so go negative
      mySerial.print("  NEG");
      motor.command(BUOY_MTR_SEL,BUOY_MTR_NEG);
    }
  } else if(change_request<deadband) {
    // below target depth, become more positive by decreasing
    // piston water volume
    if(motor.status(BUOY_MTR_SEL) & BUOY_MTR_POS) {
      mySerial.print("  pos");
    } else if(motor.status(BUOY_MTR_SEL) & BUOY_MTR_NEG) {
      mySerial.print("  OFF...");
      motor.command(BUOY_MTR_SEL,MOTOR_OFF);
    } else {
      // off so go positive
      mySerial.print("  POS");
      motor.command(BUOY_MTR_SEL,BUOY_MTR_POS);
    }
  } else {
    // within the deadband - shutoff.
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
  
  mySerial.print("buoy_G_prop="); mySerial.println(G_prop);
  mySerial.print("buoy_R_integ="); mySerial.println(R_integ);
  mySerial.print("buoy_T_deriv="); mySerial.println(T_deriv);
  mySerial.print("buoy_transit_fraction="); mySerial.println(transit_fraction);
  
  // mySerial.print("buoy_T_lowpass="); mySerial.println(T_lowpass);
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
    mySerial.println("Appears not to have hit its limit");
    motor.command(BUOY_MTR_SEL,BUOY_MTR_POS);
  }
  mySerial.println("At full positive -- armed to start mission");

  motor.command(BUOY_MTR_SEL,MOTOR_OFF);
  motor.position_reset(BUOY_MTR_SEL);
  
  // Make sure we have an up to date pressure reading
  pressure.read();
  atm_press_dPa=pressure.pressure_abs_dPa;
  mySerial.print("Atmospheric pressure recorded as ");
  mySerial.print(atm_press_dPa/1000.0f);
  mySerial.println("dbar");
  
  // Set Buoyancy state so that once the sample loop is going
  // the PID loop will begin.
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

float Buoyancy::piston_water_volume(void) {
  long pos=motor.position(BUOY_MTR_SEL);
  //        ms * ml/s * s/ms => ml
  float vol=pos*(PISTON_RATE*0.001);
  
  if ( BUOY_MTR_NEG==MOTOR_REV ) vol=BUOY_PISTON_ML-vol;
  return vol;
}

bool Buoyancy::dispatch_command(const char *cmd, const char *cmd_arg) {
  float tmp;
  if ( !strcmp(cmd,"buoy") ) {
    display();
  } else if(!strcmp(cmd,"buoy_G_prop")) {
    set_float(cmd_arg,&G_prop,"buoy_G_prop");
  } else if(!strcmp(cmd,"buoy_R_integ")) {
    set_float(cmd_arg,&R_integ,"buoy_G_prop");
  } else if(!strcmp(cmd,"buoy_T_deriv")) {
    set_float(cmd_arg,&T_deriv,"buoy_T_deriv");
  //  } else if(!strcmp(cmd,"buoy_T_lowpass")) {
  //    set_float(cmd_arg,&T_lowpass,"buoy_T_lowpass");
  } else if(!strcmp(cmd,"buoy_transit_fraction")) {
    set_float(cmd_arg,&transit_fraction,"buoy_transit_fraction");
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

typedef struct {
  int16_t a_status,b_status;
  int16_t a_sense,b_sense;
  int32_t a_position,b_position;
} full_record;

void Buoyancy::write_frame_info(Print &out) {
  out.print("('buoy_z_m','<f4'),('buoy_w_mps','<f4'),('buoy_ctrl_prop','<f4'),('buoy_ctrl_deriv','<f4'),('buoy_ctrl_integ','<f4'),('buoy_ctrl','<f4'),");
}

#define WRITE_VAR(X) write_base16(out,(uint8_t*)&(X),sizeof(X))
void Buoyancy::write_data(Print &out){
  WRITE_VAR(lpx[0]);
  WRITE_VAR(w_mps);
  WRITE_VAR(ctrl_prop);
  WRITE_VAR(ctrl_deriv);
  WRITE_VAR(ctrl_integ);
  WRITE_VAR(ctrl);
}

#endif // HAS_BUOYANCY
