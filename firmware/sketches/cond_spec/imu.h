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
  
  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);

  imu::Vector<3> euler;
  imu::Vector<3> accel;
  uint8_t cal_system, cal_gyro, cal_accel, cal_mag = 0;

};

#endif // IMU_H
