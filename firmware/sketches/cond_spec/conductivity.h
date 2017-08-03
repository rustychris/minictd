#ifndef CONDUCTIVITY_H
#define CONDUCTIVITY_H

class Conductivity {
public:  
  void init();
  void read();


  void setup();
  
  void dac_status();
  void fill_sine_buffer();
  void adc_setup();
  void adc0_setup();
  void adc1_setup(void);
  void dac_setup(void);
  void pdb_setup(void);

  void pdb_stop(void);
  void adc_stop(void);
  void dac_stop();
};

#endif // CONDUCTIVITY_H
