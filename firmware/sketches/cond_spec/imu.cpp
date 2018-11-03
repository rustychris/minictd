// IMU-seaduck interface
#include "cfg_imu.h"

#ifdef HAS_IMU
#include "imu.h"

// #include <AWire.h>
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

  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // euler.x(), .y(), .z()
  // accel.x(), .y(), .z()

  bno.getCalibration(&cal_system, &cal_gyro, &cal_accel, &cal_mag);
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
  Serial.println("    imu  # report orientation and linear acceleration");
  Serial.println("    imu_enable[=0,1] # enable/disable ");
}

void IMU::write_frame_info(Print &out) {
  out.print( "('imu_euler_deg','<f8',3),('imu_accel_m_s2','<f8',3),"
             "('cal_system','<u1'),('cal_gyro','<u1'),('cal_accel','<u1'),('cal_mag','<u1')" );
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
