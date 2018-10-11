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


class SeaDuck : public Shell
{
  uint32_t sample_interval_us;
  
public:
  // mostly globals, to lessen the pain of access from ISR
  SeaDuck(); 

  void setup();
  virtual void dispatch_command();
  virtual void help();

  Sensor *sensors[MAX_NUM_SENSORS];
  int num_sensors;

  time_t unixtime();

  void oneshot_sample();
  void continuous_sample();
  void write_header(void);
  void async_oneshot(void);
  void async_output(void);
};

extern SeaDuck logger;

void check_adc_error();

#endif // SEADUCK_H
