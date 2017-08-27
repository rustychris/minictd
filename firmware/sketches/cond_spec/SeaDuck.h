#ifndef SEADUCK_H
#define SEADUCK_H

#include <ADC.h>
// #undef PDB_CONFIG // avoid clash between ADC and Audio
// #include <utility/pdb.h>

#include "Shell.h"
#include "seaduck_cfg.h"

extern ADC *adc;

class SeaDuck : public Shell
{
public:
  // mostly globals, to lessen the pain of access from ISR
  SeaDuck(); 

  void setup();
  virtual void dispatch_command();
  virtual void help();

  void scan();
  void scan_setup();
  void scan_loop();
  void scan_cleanup();
};

extern SeaDuck seaduck;

void check_adc_error();

#endif // SEADUCK_H
