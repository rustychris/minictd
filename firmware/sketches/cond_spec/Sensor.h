/* 
   Abstact base class for individual sensors
 */
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <SdFat.h>

class Sensor {
public:
  Sensor() {
    enabled=true;
  }
  
  virtual void init(){};
  virtual void read(){};
  virtual bool dispatch_command(const char *, const char *){return false;};
  virtual void help(){};
  virtual void write_frame_info(Print &out){};
  virtual void write_data(Print &out){};

  bool enabled;
};

typedef enum { BIN_HEX, BIN_RAW } binary_format_t;
extern binary_format_t binary_format;

void write_base16(Print &out,uint8_t *buff,int count);

#endif // __SENSOR_H__
