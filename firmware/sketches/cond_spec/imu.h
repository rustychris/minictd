#ifndef IMU_H
#define IMU_H

#include "Sensor.h"
// These shouldn't strictly be needed here, but arduino build logic
// picks up library directories based on includes like this,
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU : public Sensor {
 public:
 IMU() {
    strcpy(name,"imu");
  }
  
  virtual void init();
  virtual void async_read();
  void display();
  void display_cal(void);

  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);

  imu::Vector<3> euler;
  imu::Vector<3> accel;
  // volatile imu::Vector<3> target;
  volatile int16_t target[3];
  uint8_t cal_system, cal_gyro, cal_accel, cal_mag = 0;

  void async_readEuler(void);
  void async_readEuler2(void);
  void async_readAccel(void);
  void async_readAccel2(void);
  void async_getVector(Adafruit_BNO055::adafruit_vector_type_t vector_type);
  void async_readVector1(void);
  void async_readVector2(void);
};

#endif // IMU_H
