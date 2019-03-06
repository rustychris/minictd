#include "cfg_seaduck.h"
#ifdef HAS_MOTOR

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

void Motor::async_read() {
  // no work for motor yet, but we still have to play nice
  // and keep the ball in the air.
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
      Serial.println("LIMIT"); // temporary
      a_status |= MOTOR_LIMIT;
    }
  }
  if (motor&MOTOR_B) {
    b_sense=analogRead(MOTOR_BSENSE) - b_sense_offset;
    b_current=b_decay*b_current + (1-b_decay)*b_sense*3.3/4096.0/MOTOR_BSENSE_R;
    if ( (b_status&(MOTOR_FWD|MOTOR_REV)) && (b_current<b_current_threshold) ) {
      Serial.println("LIMIT"); // temporary
      b_status |= MOTOR_LIMIT;
    }
  }
}

void Motor::display(unsigned int select) {
  if( select & DISP_ENABLE ) {
    Serial.print("motor_enable="); Serial.println( enabled );
  }

  if( (a_status!=MOTOR_OFF) || (select&DISP_OFF) ) {
    read_current(MOTOR_A);
    if(select & DISP_SENSE ) {
      Serial.print("motor_a_sense="); Serial.println(a_sense);
    }
    if(select & DISP_SENSE ) {
      Serial.print("motor_a_sense="); Serial.println(a_sense);
    }
    if(select&DISP_CURRENT) {
      Serial.print("motor_a_current="); Serial.println(a_current,3);
    }
    Serial.print("motor_a_status=");
    if( a_status&MOTOR_FWD ) Serial.print(" FWD");
    if( a_status&MOTOR_REV ) Serial.print(" REV");
    if( a_status&MOTOR_LIMIT ) Serial.print(" LIMIT");
    Serial.println();
  }
  if( (b_status!=MOTOR_OFF) || (select&DISP_OFF) ) {
    read_current(MOTOR_B);
    if(select & DISP_SENSE ) {
      Serial.print("motor_b_sense="); Serial.println(b_sense);
    }
    if(select&DISP_CURRENT) {
      Serial.print("motor_b_current="); Serial.println(b_current,3);
    }
    Serial.print("motor_b_status=");
    if( b_status&MOTOR_FWD ) Serial.print(" FWD");
    if( b_status&MOTOR_REV ) Serial.print(" REV");
    if( b_status&MOTOR_LIMIT ) Serial.print(" LIMIT");
    Serial.println();
  }
}

int Motor::wait_and_stop() {
  int condition=0;
  
  // flush input buffer, then stop on any input
  while(Serial.available() ) { Serial.read(); }

  while(1) {
    if( Serial.available() ) {
      uint8_t c=Serial.read();
      break;
    } else {
      display(DISP_CURRENT);
      // that gets us fresh limit status
      if(a_status&MOTOR_LIMIT) {
        Serial.println("# Motor A detected limit switch");
        command(MOTOR_A,MOTOR_OFF);
        condition=MOTOR_LIMIT;
      }
      if(b_status&MOTOR_LIMIT) {
        Serial.println("# Motor B detected limit switch");
        command(MOTOR_B,MOTOR_OFF);
        condition=MOTOR_LIMIT;
      }
      if( (a_status==MOTOR_OFF) && (b_status==MOTOR_OFF) ){
        Serial.println("# All motors off");
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
    if ( !enabled ){ Serial.println("Motor is not enabled"); }
    else {
      command(MOTOR_A,MOTOR_FWD);
      wait_and_stop();
    }
  } else if(!strcmp(cmd,"motor_a_rev")) {
    if ( !enabled ){ Serial.println("Motor is not enabled"); }
    else {
      command(MOTOR_A,MOTOR_REV);
      wait_and_stop();
    }
  } else if(!strcmp(cmd,"motor_b_fwd")) {
    if ( !enabled ){ Serial.println("Motor is not enabled"); }
    else {
      command(MOTOR_B,MOTOR_FWD);
      wait_and_stop();
    }
  } else if(!strcmp(cmd,"motor_b_rev")) {
    if ( !enabled ) { Serial.println("Motor is not enabled"); }
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
      Serial.print("motor_enable="); Serial.println( enabled );
    }
  } else {
    return false;
  }
  return true;
}


void Motor::help() {
  Serial.println("  Motor");
  Serial.println("    motor               # report sense and drive");
  Serial.println("    motor_enable        # report motor enable status");
  Serial.println("    motor_enable=[0,1]  # disarm/arm motor");
  Serial.println("    motor_off           # all motors off");
  Serial.println("    motor_a_fwd         # motor A forward until keypress");
  Serial.println("    motor_a_rev         # motor A reverse until keypress");
  Serial.println("    motor_b_fwd         # motor B forward until keypress");
  Serial.println("    motor_b_rev         # motor B reverse until keypress");
}
  
void Motor::write_frame_info(Print &out) { }
void Motor::write_data(Print &out){ }

#endif // HAS_MOTOR
