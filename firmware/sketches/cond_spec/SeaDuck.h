#ifndef SEADUCK_H
#define SEADUCK_H

#include <ADC.h>
// #undef PDB_CONFIG // avoid clash between ADC and Audio
// #include <utility/pdb.h>

#include "Shell.h"
#include <TimeLib.h>

#include "Sensor.h"

#include "seaduck_cfg.h"

extern ADC *adc;

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
};

extern SeaDuck logger;

void check_adc_error();

#endif // SEADUCK_H
