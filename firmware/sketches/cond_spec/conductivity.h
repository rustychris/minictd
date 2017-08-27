#ifndef CONDUCTIVITY_H
#define CONDUCTIVITY_H

class Conductivity {
public:
  Conductivity();
  
  void init();
  void read();

  void scan_setup();
  
  void dac_status();
  void fill_sine_buffer();
  void adc_setup();
  void adc0_setup();
  void adc1_setup(void);
  void dac_setup(void);
  void pdb_setup(void);

  void scan_cleanup();
  void pdb_stop(void);
  void adc_stop(void);
  void dac_stop();

  void wait_for_scan();
  void scan();
  void scan_dump();
  void scan_reduce();
  
  bool dispatch_command(const char *, const char *);
  void help();

  float reading; // would it be better for this to be a scaled int?
};

#endif // CONDUCTIVITY_H
