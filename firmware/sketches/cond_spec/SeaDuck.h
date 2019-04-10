#ifndef SEADUCK_H
#define SEADUCK_H

#include "cfg_seaduck.h"

#if defined(HAS_CONDUCTIVITY) || defined(HAS_NTC)
// at the moment cond/ntc are synonymous with teensy, though
// that may change in the future
#define USE_TEENSY_ADC
#include <ADC.h>
extern ADC *adc;
#endif

#include "Shell.h"
#include <TimeLib.h>

#include "Sensor.h"


typedef enum { STOP_NONE=0,STOP_DURATION=1,STOP_KEY=2 } stop_condition_t;

class SeaDuck : public Shell
{
  uint32_t sample_interval_us;
  
public:
  // mostly globals, to lessen the pain of access from ISR
  SeaDuck(); 

  void setup();
  virtual void dispatch_command(const char *, const char *);
  virtual void help();

  Sensor *sensors[MAX_NUM_SENSORS];
  int num_sensors;

  time_t unixtime();

  void oneshot_sample();
  stop_condition_t continuous_sample(long duration_ms=0);
  void write_header(void);
  void async_oneshot(void);
  void async_output(void);
};

extern SeaDuck logger;

void check_adc_error();

#endif // SEADUCK_H
