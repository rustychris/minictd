// IMU-seaduck interface
#include "cfg_seaduck.h"

#ifdef HAS_IMU
#include <AWire.h>
#include "imu.h"
// #define Wire AWire // this is done in my Adafruit_BNO055.h

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

/* large parts of this taken from the rawdata.ino example from adafruit */

void IMU::init(){
  /* Initialise the sensor */
  if(!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
  delay(1000); // wow - that's a long time.  maybe doesn't have to be so long??

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  // Serial.print("# BNO055 Current Temperature: ");
  // Serial.print(temp);
  // Serial.println(" C");

  bno.setExtCrystalUse(true);

  // Serial.println("# BNO055 Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

void IMU::async_read() {
  // calling convention for async_read is that the callback stack should be
  // popped after the read is complete.

#ifndef ASYNC_I2C
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // euler.x(), .y(), .z()
  // accel.x(), .y(), .z()

  bno.getCalibration(&cal_system, &cal_gyro, &cal_accel, &cal_mag);
  pop_fn_and_call();
#else
  push_fn(this,(SensorFn)&IMU::async_readEuler);
  push_fn(this,(SensorFn)&IMU::async_readAccel);
  pop_fn_and_call();
#endif
}

// Status: seems to read the first euler angle okay, but the other
// angles are crap, and acceleration is all zero.

void IMU::async_readEuler(void) {
  // euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // Serial.println("async_readEuler");
  push_fn(this, (SensorFn)&IMU::async_readEuler2);
  async_getVector(Adafruit_BNO055::VECTOR_EULER);
}

void IMU::async_readEuler2(void) {
  //case VECTOR_EULER:
  /* 1 degree = 16 LSB */
  // Serial.println("async_readEuler2");
  for(int i=0;i<3;i++)
    euler[i]=0.0625*target[i];// 1/16.
  pop_fn_and_call();
}

void IMU::async_readAccel(void) {
  // /Serial.println("async_readAccel");
  push_fn(this, (SensorFn)&IMU::async_readAccel2);
  async_getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

void IMU::async_readAccel2(void) {
  // case VECTOR_ACCELEROMETER:
  // case VECTOR_LINEARACCEL:
  // case VECTOR_GRAVITY:
  /* 1m/s^2 = 100 LSB */
  // Serial.println("async_readAccel2");
  // Serial.print("accel[0] ");
  // Serial.println(accel[0]);
  for(int i=0;i<3;i++)
    accel[i]=0.01*target[i];// 1/100
  
  pop_fn_and_call();
}


void IMU::async_getVector(Adafruit_BNO055::adafruit_vector_type_t vector_type) {
  // readLen((adafruit_bno055_reg_t)vector_type, buffer, 6);
  AWire.beginTransmission(bno._address);
  AWire.write((uint8_t)vector_type);
  push_fn(this,(SensorFn)&IMU::async_readVector1);
  AWire.onTransmitDone(pop_fn_and_call);
  AWire.sendTransmission();
}

void IMU::async_readVector1(void) {
  AWire.onTransmitDone(NULL);
  push_fn(this,(SensorFn)&IMU::async_readVector2);
  AWire.onReqFromDone(pop_fn_and_call); // I think this is what is not getting called.
  // 2 bytes per each of 3 components:
  AWire.sendRequest(bno._address, 6);
}

void IMU::async_readVector2(void) {
  // 2 bytes per each of 3 components:
  AWire.onReqFromDone(NULL);
  // Serial.println("hit readVector2");
  uint8_t lsb,msb;
  int16_t combined;
  for (uint8_t i = 0; i < 3; i++) {
    lsb = AWire.read();
    msb = AWire.read();
    combined=( ((int16_t)lsb) | (((int16_t)msb) << 8) );
    // Serial.print("  combined: ");
    // Serial.println(combined);
    target[i] = combined ;
    // Serial.print("  in target: ");
    // Serial.println(target[i]);
  }
  pop_fn_and_call();
}

void IMU::display(){
  read();
  
  Serial.print("euler_angles=[");
  Serial.print(euler.x());
  Serial.print(",");
  Serial.print(euler.y());
  Serial.print(",");
  Serial.print(euler.z());
  Serial.println("]");

  Serial.print("linear_accel=[");
  Serial.print(accel.x());
  Serial.print(",");
  Serial.print(accel.y());
  Serial.print(",");
  Serial.print(accel.z());
  Serial.println("]");
}

void IMU::display_cal(void) {
  bno.getCalibration(&cal_system,&cal_gyro,&cal_accel,&cal_mag);

  Serial.print("cal_system=");
  Serial.println(cal_system,DEC);
  Serial.print("cal_gyro=");
  Serial.println(cal_gyro,DEC);
  Serial.print("cal_accel=");
  Serial.println(cal_accel,DEC);
  Serial.print("cal_mag=");
  Serial.println(cal_mag,DEC);
}

bool IMU::dispatch_command(const char *cmd, const char *cmd_arg) {
  if ( !strcmp(cmd,"imu") ) {
    display();
  } else if ( !strcmp(cmd,"imu_cal") ) {
    display_cal();
  } else if ( strcmp(cmd,"imu_enable")==0 ) {
    if(cmd_arg) {
      enabled=(bool)atoi(cmd_arg);
    } else {
      Serial.print("imu_enable="); Serial.println( enabled );
    }
  } else {
    return false;
  }
  return true;
}

void IMU::help() {
  Serial.println("  IMU");
  Serial.println("    imu              # report orientation and linear acceleration");
  Serial.println("    imu_cal          # report sensor calibration state");
  Serial.println("    imu_enable[=0,1] # enable/disable ");
}

void IMU::write_frame_info(Print &out) {
  out.print( "('imu_euler_deg','<f8',3),('imu_accel_m_s2','<f8',3),"
             // "('cal_system','<u1'),('cal_gyro','<u1'),('cal_accel','<u1'),('cal_mag','<u1'),"
             );
}

typedef struct {
  double euler_x, euler_y, euler_z;
  double accel_x, accel_y, accel_z;
  uint8_t cal_system, cal_gyro, cal_accel, cal_mag;
} full_record;

void IMU::write_data(Print &out){
  full_record rec;
  rec.euler_x=euler.x();
  rec.euler_y=euler.y();
  rec.euler_z=euler.z();

  rec.accel_x=accel.x();
  rec.accel_y=accel.y();
  rec.accel_z=accel.z();
  rec.cal_system=cal_system;
  rec.cal_gyro=cal_gyro;
  rec.cal_accel=cal_accel;
  rec.cal_mag=cal_mag;

  write_base16(out,(uint8_t*)&rec,sizeof(rec));
}

#endif // HAS_IMU
