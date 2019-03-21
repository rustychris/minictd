#include "cfg_seaduck.h"
#ifdef HAS_MOTOR

#include "serialmux.h"
#include "motor.h"

void Motor::init() {
  analogReadResolution(12);
  pinMode(MOTOR_A1,OUTPUT);
  pinMode(MOTOR_A2,OUTPUT);
  pinMode(MOTOR_B1,OUTPUT);
  pinMode(MOTOR_B2,OUTPUT);
  pinMode(MOTOR_nSLEEP,OUTPUT);
  
  pinMode(MOTOR_ASENSE,INPUT);
  pinMode(MOTOR_BSENSE,INPUT);

  // Sensor will default to enabled=True, but
  // for the motor safer to disable, require
  // user to enable.
  disable();

  a_sense_offset=analogRead(MOTOR_ASENSE);
  b_sense_offset=analogRead(MOTOR_BSENSE);
  a_position=-99999; // no knowledge of absolute position
  b_position=-99999; // 
}

void Motor::disable(void){
  digitalWrite(MOTOR_nSLEEP,0);
  all_off();
  enabled=false;
}

void Motor::enable(void) {
  digitalWrite(MOTOR_nSLEEP,1);
  enabled=true;
}

void Motor::all_off(void){
  command(MOTOR_A|MOTOR_B,MOTOR_OFF);
}

// handle requests for state transitions 
void Motor::command(int motor,int cmd) {
  update_position();
  
  if (cmd==MOTOR_OFF) {
    if(motor&MOTOR_A) {
      digitalWrite(MOTOR_A1,0);
      digitalWrite(MOTOR_A2,0);
      a_status=MOTOR_OFF;
    }
    if (motor&MOTOR_B) {
      digitalWrite(MOTOR_B1,0);
      digitalWrite(MOTOR_B2,0);
      b_status=MOTOR_OFF;
    } 
  } else if (cmd==MOTOR_FWD) {
    if(motor&MOTOR_A) {
      digitalWrite(MOTOR_A1,1);
      digitalWrite(MOTOR_A2,0);
      a_status=MOTOR_FWD;
    }
    if(motor&MOTOR_B) {
      digitalWrite(MOTOR_B1,1);
      digitalWrite(MOTOR_B2,0);
      b_status=MOTOR_FWD;
    }
  } else if (cmd==MOTOR_REV) {
    if(motor&MOTOR_A) {
      digitalWrite(MOTOR_A1,0);
      digitalWrite(MOTOR_A2,1);
      a_status=MOTOR_REV;
    }
    if(motor&MOTOR_B) {
      digitalWrite(MOTOR_B1,0);
      digitalWrite(MOTOR_B2,1);
      b_status=MOTOR_REV;
    }
  }  
}

void Motor::update_position(void) {
  // check current draw, but only for motors known to be on.
  if ( a_status&MOTOR_ON ) {
    read_current(MOTOR_A);
    // update integral time
    if ( a_status==MOTOR_FWD ) { // NOT at limit
      a_position+=position_elapsed;
    } else if ( a_status==MOTOR_REV ) { // NOT at limit
      a_position-=position_elapsed;
    } else if ( a_status==MOTOR_REV|MOTOR_LIMIT ) {
      a_position=0; // established absolute zero.
    }
    // no update for MOTOR_FWD|MOTOR_LIMIT
  }
  if ( b_status&MOTOR_ON ) {
    read_current(MOTOR_B);
    if ( b_status==MOTOR_FWD ) {
      b_position+=position_elapsed;
    } else if ( b_status==MOTOR_REV ) {
      b_position-=position_elapsed;
    } else if ( b_status==MOTOR_REV|MOTOR_LIMIT ) {
      b_position=0;
    }
  }
  position_elapsed=0;
}

void Motor::async_read() {
  // in the future may include checks for current draw here
  // to be logged

  update_position();
  
  pop_fn_and_call();
}

void Motor::read_current(int motor){
  // read motor current for one or both, and update limit status
  // for motors currently on.
  if (motor&MOTOR_A) {
    // this comes in as counts, and defaults to 10 bits over
    // the 0..3.3V range.
    a_sense=analogRead(MOTOR_ASENSE) - a_sense_offset;
    a_current=a_decay*a_current + (1-a_decay)*a_sense*3.3/4096.0/MOTOR_ASENSE_R;
    if (  (a_status&(MOTOR_FWD|MOTOR_REV)) && (a_current<a_current_threshold) ) {
      mySerial.println("LIMIT"); // temporary
      a_status |= MOTOR_LIMIT;
    }
  }
  if (motor&MOTOR_B) {
    b_sense=analogRead(MOTOR_BSENSE) - b_sense_offset;
    b_current=b_decay*b_current + (1-b_decay)*b_sense*3.3/4096.0/MOTOR_BSENSE_R;
    if ( (b_status&(MOTOR_FWD|MOTOR_REV)) && (b_current<b_current_threshold) ) {
      mySerial.println("LIMIT"); // temporary
      b_status |= MOTOR_LIMIT;
    }
  }
}

void Motor::display(unsigned int select) {
  update_position();
  
  if( select & DISP_ENABLE ) {
    mySerial.print("motor_enable="); mySerial.println( enabled );
  }

  if( (a_status!=MOTOR_OFF) || (select&DISP_OFF) ) {
    read_current(MOTOR_A);
    if(select&DISP_SENSE ) {
      mySerial.print("motor_a_sense="); mySerial.println(a_sense);
    }
    if(select&DISP_CURRENT) {
      mySerial.print("motor_a_current="); mySerial.println(a_current,3);
    }
    if(select&DISP_POSITION) {
      mySerial.print("motor_a_position="); mySerial.println(a_position);
    }
    mySerial.print("motor_a_status=");
    if( a_status&MOTOR_FWD ) mySerial.print(" FWD");
    if( a_status&MOTOR_REV ) mySerial.print(" REV");
    if( a_status&MOTOR_LIMIT ) mySerial.print(" LIMIT");
    mySerial.println();
  }
  if( (b_status!=MOTOR_OFF) || (select&DISP_OFF) ) {
    read_current(MOTOR_B);
    if(select & DISP_SENSE ) {
      mySerial.print("motor_b_sense="); mySerial.println(b_sense);
    }
    if(select&DISP_CURRENT) {
      mySerial.print("motor_b_current="); mySerial.println(b_current,3);
    }
    if(select&DISP_POSITION) {
      mySerial.print("motor_b_position="); mySerial.println(b_position);
    }
    mySerial.print("motor_b_status=");
    if( b_status&MOTOR_FWD ) mySerial.print(" FWD");
    if( b_status&MOTOR_REV ) mySerial.print(" REV");
    if( b_status&MOTOR_LIMIT ) mySerial.print(" LIMIT");
    mySerial.println();
  }
}

int Motor::wait_and_stop() {
  int condition=0;
  
  // flush input buffer, then stop on any input
  while(mySerial.available() ) { mySerial.read(); }

  // Prop up the motor current so the lowpass has a chance to catch up
  if (a_status&(MOTOR_FWD|MOTOR_REV)) {
    mySerial.println("# Padding up motor A current");
    a_current=10*a_current_threshold;
  }
  if (b_status&(MOTOR_FWD|MOTOR_REV)) {
    mySerial.println("# Padding up motor B current");
    b_current=10*b_current_threshold;
  }

  while(1) {
    if( mySerial.available() ) {
      uint8_t c=mySerial.read();
      break;
    } else {
      display(DISP_CURRENT);
      // that gets us fresh limit status
      if(a_status&MOTOR_LIMIT) {
        mySerial.println("# Motor A detected limit switch");
        command(MOTOR_A,MOTOR_OFF);
        condition=MOTOR_LIMIT;
      }
      if(b_status&MOTOR_LIMIT) {
        mySerial.println("# Motor B detected limit switch");
        command(MOTOR_B,MOTOR_OFF);
        condition=MOTOR_LIMIT;
      }
      if( (a_status==MOTOR_OFF) && (b_status==MOTOR_OFF) ){
        mySerial.println("# All motors off");
        break;
      }
      delay(40);
    }
  }
  all_off(); // partially redundant, but that's fine.
  return condition; 
}

bool Motor::dispatch_command(const char *cmd, const char *cmd_arg) {
  if ( !strcmp(cmd,"motor") ) {
    display();
  } else if(!strcmp(cmd,"motor_a_fwd")) {
    if ( !enabled ){ mySerial.println("Motor is not enabled"); }
    else {
      command(MOTOR_A,MOTOR_FWD);
      wait_and_stop();
    }
  } else if(!strcmp(cmd,"motor_a_rev")) {
    if ( !enabled ){ mySerial.println("Motor is not enabled"); }
    else {
      command(MOTOR_A,MOTOR_REV);
      wait_and_stop();
    }
  } else if(!strcmp(cmd,"motor_b_fwd")) {
    if ( !enabled ){ mySerial.println("Motor is not enabled"); }
    else {
      command(MOTOR_B,MOTOR_FWD);
      wait_and_stop();
    }
  } else if(!strcmp(cmd,"motor_b_rev")) {
    if ( !enabled ) { mySerial.println("Motor is not enabled"); }
    else {
      command(MOTOR_B,MOTOR_REV);
      wait_and_stop();
    }
  } else if(!strcmp(cmd,"motor_off")) {
    all_off();
  } else if(!strcmp(cmd,"motor_enable")) {
    if(cmd_arg) {
      if ( atoi(cmd_arg) ) {
        enable();
      } else {
        disable();
      }
    } else {
      mySerial.print("motor_enable="); mySerial.println( enabled );
    }
  } else {
    return false;
  }
  return true;
}


void Motor::help() {
  mySerial.println("  Motor");
  mySerial.println("    motor               # report sense and drive");
  mySerial.println("    motor_enable        # report motor enable status");
  mySerial.println("    motor_enable=[0,1]  # disarm/arm motor");
  mySerial.println("    motor_off           # all motors off");
  mySerial.println("    motor_a_fwd         # motor A forward until keypress");
  mySerial.println("    motor_a_rev         # motor A reverse until keypress");
  mySerial.println("    motor_b_fwd         # motor B forward until keypress");
  mySerial.println("    motor_b_rev         # motor B reverse until keypress");
}
  
void Motor::write_frame_info(Print &out) {
  out.print("('motor_status','<i2',2),('motor_sense','<i2',2),");
}

typedef struct {
  int16_t a_status,b_status;
  int16_t a_sense,b_sense;
} full_record;

void Motor::write_data(Print &out){
  full_record rec;
  rec.a_status=a_status;
  rec.b_status=b_status;
  rec.a_sense=(int16_t)a_sense;
  rec.b_sense=(int16_t)b_sense;
  
  write_base16(out,(uint8_t*)&rec,sizeof(rec));
}

#endif // HAS_MOTOR
