/*
   Abstact base class for individual sensors
 */
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <SdFat.h>

#ifdef ARDUINO_ARCH_SAMD
// zero:
#include "samd_IntervalTimer.h"
#define IntervalTimer SAMD_IntervalTimer
#endif

extern IntervalTimer sensorTimer;

class Sensor {
public:
  char name[25];
  Sensor() {
    enabled=true;
    strcpy(name,"unknown");
  }
  
  virtual void init(){};
  void read();
  virtual void async_read(){};

  // handle user commands, or return false if not handled.
  // this must be reentrant, so that compound commands can be used.
  virtual bool dispatch_command(const char *, const char *){return false;};
  virtual void help(){};
  virtual void write_frame_info(Print &out){};
  virtual void write_data(Print &out){};
  // called just before the sampling loop starts
  virtual void enter_sample_loop(void) {};
  // called when the sampling loop exits.  All async tasks will be completed,
  // but storage has not been finalized, so a small amount of additional data
  // can be written.
  virtual void exit_sample_loop(void) {};

  volatile int busy;
  void clear_busy(void);
  void push_busy(void);
  
  volatile bool enabled;
};

typedef enum { BIN_HEX, BIN_RAW } binary_format_t;
extern binary_format_t binary_format;

void write_base16(Print &out,uint8_t *buff,int count);

// cheapo function stack for ISRs

// stack of void (*)(void)
typedef void (Sensor::*SensorFn)(void);

typedef struct {
  SensorFn fn;
  Sensor *s;
} SensorClosure; // yeah yeah, it's not a closure.

#define FN_STACK_MAX 20
void push_fn(Sensor *,SensorFn fn);
void pop_fn(void); 
void pop_fn_and_call(void);
int stack_size();

#endif // __SENSOR_H__
