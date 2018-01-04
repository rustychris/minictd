/* 
   Abstact base class for individual sensors
 */
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <SdFat.h>

extern IntervalTimer sensorTimer;

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

// wacky function stack for ISRs

// stack of void (*)(void)
typedef  void (Sensor::*SensorFn)(void);
// typedef void (*SensorFn)(Sensor*);

typedef struct {
  SensorFn fn;
  Sensor *s;
} SensorClosure; // yeah yeah, it's not a closure.

#define FN_STACK_MAX 20
void push_fn(Sensor *,SensorFn fn);
void pop_fn(void); 
void pop_fn_and_call(void);

#endif // __SENSOR_H__
