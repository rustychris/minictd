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
  digitalWrite(MOTOR_A1,0);
  digitalWrite(MOTOR_A2,0);
  digitalWrite(MOTOR_B1,0);
  digitalWrite(MOTOR_B2,0);
}

void Motor::async_read() { }

void Motor::display() {
  int a_sense=analogRead(MOTOR_ASENSE) - a_sense_offset;
  int b_sense=analogRead(MOTOR_BSENSE) - b_sense_offset;

  Serial.print("motor_enable="); Serial.println( enabled );
  
  Serial.print("motor_a_sense="); Serial.println(a_sense);
  Serial.print("motor_b_sense="); Serial.println(b_sense);
  // this comes in as counts, and defaults to 10 bits over
  // the 0..3.3V range.
  
  Serial.print("motor_a_current="); Serial.println(a_sense*3.3/4096.0/MOTOR_ASENSE_R,3);
  Serial.print("motor_b_current="); Serial.println(b_sense*3.3/4096.0/MOTOR_BSENSE_R,3);
}

bool Motor::dispatch_command(const char *cmd, const char *cmd_arg) {
  if ( !strcmp(cmd,"motor") ) {
    display();
  } else if(!strcmp(cmd,"motor_a_fwd")) {
    digitalWrite(MOTOR_A1,1);
    digitalWrite(MOTOR_A2,0);
  } else if(!strcmp(cmd,"motor_a_rev")) {
    digitalWrite(MOTOR_A1,0);
    digitalWrite(MOTOR_A2,1);
  } else if(!strcmp(cmd,"motor_b_fwd")) {
    digitalWrite(MOTOR_B1,1);
    digitalWrite(MOTOR_B2,0);
  } else if(!strcmp(cmd,"motor_b_rev")) {
    digitalWrite(MOTOR_B1,0);
    digitalWrite(MOTOR_B2,1);
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
  Serial.println("    motor  # report sense and drive");
}
  
void Motor::write_frame_info(Print &out) { }
void Motor::write_data(Print &out){ }

#endif // HAS_MOTOR
