#ifndef SEADUCK_H
#define SEADUCK_H

#include <ADC.h>
#include <DMAChannel.h> // used to be in quotes
// #undef PDB_CONFIG // avoid clash between ADC and Audio
// #include <utility/pdb.h>

#include "Shell.h"
#include "seaduck_cfg.h"

class SeaDuck : public Shell
{
public:
  // mostly globals, to lessen the pain of access from ISR
  SeaDuck(); 

  void dac_init();
  void dac_status();
  void fill_sine_buffer();
  void adc_init();
  void adc_setup();
  void adc0_setup();
  void adc1_setup(void);
  void dac_setup(void);
  void pdb_setup(void);

  void pdb_stop(void);
  void adc0_stop(void);
  void adc1_stop(void);
  void adc_stop(void);
  void dac_stop();

  void setup();
  virtual void dispatch_command();
  virtual void help();

  void scan();
  void scan_setup();
  void scan_loop();
  void scan_cleanup();
};

extern SeaDuck seaduck;

#endif // SEADUCK_H
