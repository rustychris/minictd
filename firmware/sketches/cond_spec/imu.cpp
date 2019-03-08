// IMU-seaduck interface
#include "cfg_seaduck.h"

#ifdef HAS_IMU
#include <AWire.h>
#include "serialmux.h"

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
      mySerial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
  delay(1000); // wow - that's a long time.  maybe doesn't have to be so long??

  int8_t temp = bno.getTemp(); // necessary?  probably not.
  bno.setExtCrystalUse(true);
}

void IMU::async_read() {
  // calling convention for async_read is that the callback stack should be
  // popped after the read is complete.

#ifndef ASYNC_I2C
  euler=bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  accel=bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  if(read_mag) {
    magnet=bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  }

  bno.getCalibration(&cal_system, &cal_gyro, &cal_accel, &cal_mag);
  pop_fn_and_call();
#else
  push_fn(this,(SensorFn)&IMU::async_readEuler);
  push_fn(this,(SensorFn)&IMU::async_readAccel);
  if(read_mag) {
    push_fn(this,(SensorFn)&IMU::async_readMagnet);
  }
  pop_fn_and_call();
#endif
}

// Status: seems to read the first euler angle okay, but the other
// angles are crap, and acceleration is all zero.

void IMU::async_readEuler(void) {
  // euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // mySerial.println("async_readEuler");
  push_fn(this, (SensorFn)&IMU::async_readEuler2);
  async_getVector(Adafruit_BNO055::VECTOR_EULER);
}

void IMU::async_readEuler2(void) {
  //case VECTOR_EULER:
  /* 1 degree = 16 LSB */
  // mySerial.println("async_readEuler2");
  for(int i=0;i<3;i++)
    euler[i]=0.0625*target[i];// 1/16.
  pop_fn_and_call();
}

void IMU::async_readAccel(void) {
  // /mySerial.println("async_readAccel");
  push_fn(this, (SensorFn)&IMU::async_readAccel2);
  async_getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

void IMU::async_readAccel2(void) {
  // case VECTOR_ACCELEROMETER:
  // case VECTOR_LINEARACCEL:
  // case VECTOR_GRAVITY:
  /* 1m/s^2 = 100 LSB */
  // mySerial.println("async_readAccel2");
  // mySerial.print("accel[0] ");
  // mySerial.println(accel[0]);
  for(int i=0;i<3;i++)
    accel[i]=0.01*target[i];// 1/100
  
  pop_fn_and_call();
}

void IMU::async_readMagnet(void) {
  push_fn(this, (SensorFn)&IMU::async_readMagnet2);
  async_getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
}

void IMU::async_readMagnet2(void) {
  // case VECTOR_MAGNETOMETER:
  /* 1uT = 16 LSB */
  // xyz[0] = ((double)x)/16.0;
  // xyz[1] = ((double)y)/16.0;
  // xyz[2] = ((double)z)/16.0;
  
  for(int i=0;i<3;i++)
    magnet[i]=target[i]/16.0;
  
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
  // mySerial.println("hit readVector2");
  uint8_t lsb,msb;
  int16_t combined;
  for (uint8_t i = 0; i < 3; i++) {
    lsb = AWire.read();
    msb = AWire.read();
    combined=( ((int16_t)lsb) | (((int16_t)msb) << 8) );
    // mySerial.print("  combined: ");
    // mySerial.println(combined);
    target[i] = combined ;
    // mySerial.print("  in target: ");
    // mySerial.println(target[i]);
  }
  pop_fn_and_call();
}

void IMU::watch() {
  while(mySerial.available()) {mySerial.read();}

  while(!mySerial.available()) {
    display();
  }
  mySerial.read();
}

void IMU::display(){
  int col=0;
  
  read();
  
  col+=mySerial.print("euler_angles=[");
  col+=mySerial.print(euler.x());
  col+=mySerial.print(",");
  col+=mySerial.print(euler.y());
  col+=mySerial.print(",");
  col+=mySerial.print(euler.z());
  col+=mySerial.print("]  ");

  while(col<40) col+=mySerial.print(" ");
  col=0;
  col+=mySerial.print("linear_accel=[");
  col+=mySerial.print(accel.x());
  col+=mySerial.print(",");
  col+=mySerial.print(accel.y());
  col+=mySerial.print(",");
  col+=mySerial.print(accel.z());
  col+=mySerial.print("]  ");

  if(read_mag) {
    while(col<40) col+=mySerial.print(" ");
    col=0;
    col+=mySerial.print("magnetometer=[");
    col+=mySerial.print(magnet.x());
    col+=mySerial.print(",");
    col+=mySerial.print(magnet.y());
    col+=mySerial.print(",");
    col+=mySerial.print(magnet.z());
    col+=mySerial.print("]");
  }
  
  mySerial.println();
}

void IMU::display_cal(void) {
  bno.getCalibration(&cal_system,&cal_gyro,&cal_accel,&cal_mag);

  mySerial.print("cal_system=");
  mySerial.println(cal_system,DEC);
  mySerial.print("cal_gyro=");
  mySerial.println(cal_gyro,DEC);
  mySerial.print("cal_accel=");
  mySerial.println(cal_accel,DEC);
  mySerial.print("cal_mag=");
  mySerial.println(cal_mag,DEC);
}

bool IMU::dispatch_command(const char *cmd, const char *cmd_arg) {
  if ( !strcmp(cmd,"imu") ) {
    display();
  } else if ( !strcmp(cmd,"imu_cal") ) {
    display_cal();
  } else if ( !strcmp(cmd,"imu_watch")) {
    watch();
  } else if ( strcmp(cmd,"imu_enable")==0 ) {
    if(cmd_arg) {
      enabled=(bool)atoi(cmd_arg);
    } else {
      mySerial.print("imu_enable="); mySerial.println( enabled );
    }
  } else if ( strcmp(cmd,"imu_magnet")==0 ) {
    if(cmd_arg) {
      read_mag=(bool)atoi(cmd_arg);
    } else {
      mySerial.print("imu_magnet="); mySerial.println( read_mag );
    }
  } else {
    return false;
  }
  return true;
}

void IMU::help() {
  mySerial.println("  IMU");
  mySerial.println("    imu              # report orientation and linear acceleration");
  mySerial.println("    imu_cal          # report sensor calibration state");
  mySerial.println("    imu_enable[=0,1] # enable/disable ");
  mySerial.println("    imu_magnet[=0,1]    # enable/disable reading of magnetometer");
}

void IMU::write_frame_info(Print &out) {
  out.print( "('imu_euler_deg','<f4',3),('imu_accel_m_s2','<f4',3),"
             "('cal_system','<u1'),('cal_gyro','<u1'),('cal_accel','<u1'),('cal_mag','<u1'),"
             );
}

typedef struct {
  float euler_x, euler_y, euler_z;
  float accel_x, accel_y, accel_z;
  // try to avoid struct packing issues by putting these in an array
  uint8_t cal[4]; // cal_system, cal_gyro, cal_accel, cal_mag;
} full_record;

void IMU::write_data(Print &out){
  full_record rec;
  rec.euler_x=(float)euler.x();
  rec.euler_y=(float)euler.y();
  rec.euler_z=(float)euler.z();

  rec.accel_x=(float)accel.x();
  rec.accel_y=(float)accel.y();
  rec.accel_z=(float)accel.z();
  rec.cal[0]=cal_system;
  rec.cal[1]=cal_gyro;
  rec.cal[2]=cal_accel;
  rec.cal[3]=cal_mag;

  write_base16(out,(uint8_t*)&rec,sizeof(rec));
}

#endif // HAS_IMU
