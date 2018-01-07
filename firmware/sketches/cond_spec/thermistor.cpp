//-------------------Thermistor-------------//
#include "seaduck_cfg.h"
#include "SeaDuck.h"
#include "thermistor.h"

void Thermistor::init() {
  pinMode(NTC_SENSE,INPUT);
}
volatile int dbridge;

void Thermistor::async_read() {
  float dratio;
  float R;

  adc->setAveraging(32,NTC_ADC);
  
  // this could call wait_for_cal() if calibration is underway.
  // generally should not.  But it looks like wait_for_cal()
  // is safe.
    
  push_fn(this,(SensorFn)&Thermistor::async_read_result);
  adc->enableInterrupts(NTC_ADC);

  dbridge=0; // to know if it fails
    
  adc->startSingleRead(NTC_SENSE,NTC_ADC);
}

// Is it really this simple?
// kinetis.h references adc0_isr() and adc1_isr()
// Currently thermistor is the only code that cares about
// these, though if battery sampling is included, that might
// change, at which point these would be better off in
// Sensor.cpp
void adc0_isr(void) {
  pop_fn_and_call();
}

void adc1_isr(void) {
  pop_fn_and_call();
}

void Thermistor::async_read_result(void)
{
  float dratio;
  float R;

  adc->disableInterrupts(NTC_ADC);
  // limitation of the ADC API, we have to cast single-ended
  // 16 bit readings to unsigned:
  dbridge=(uint16_t)adc->readSingle(NTC_ADC);

  // check_adc_error();
  
  // A0 is now referenced to 3.3v/2
  // the bridge output is 3.3 * Rref/(Rntc+Rref)
  // because the NTC is on top in the divider.
  // the output of the inamp is (in-3.3/2) * 10 + 3.3/2
  // dbridge = 3.3* ( 1/2 + NTC_GAIN * (Rref/(Rntc+Rref) - 1/2))
  // or taking 3.3 as Aref, then we normalize ADC unipolar to [0,1]
  
  //  1/2 + NTC_GAIN * (Rref/(Rntc+Rref) - 1/2)
  dratio = (float)dbridge / (float)(1<<16); // normalized to [0,1]
  // The first 0.5 is because the inamp's output is referenced to
  // 0.5*3.3.  The second 0.5 is because the negating input of the 
  // in amp is *also* set to 0.5*3.3.
  dratio = (dratio - 0.5) / NTC_GAIN + 0.5; // Rref / (Rntc+Rref)
  // dratio = Rref / (Rntc+Rref) - 1/2
  // 1/dratio -1 = Rntc/Rref
  R= NTC_R_REF*(1/dratio -1);
  
  //fiction!
  // reading=23.0 - (R-108000)*0.0005;
  // slightly refined but still not true calibration
  reading=23.0 - (R-108000)*0.00017;

  pop_fn_and_call();
}

bool Thermistor::dispatch_command(const char *cmd, const char *cmd_arg) {
  if( strcmp(cmd,"temperature")==0 ) {
    read();
    Serial.print("temperature=");
    Serial.println(reading);
  } else if ( strcmp(cmd,"temperature_enable")==0 ) {
    if(cmd_arg) {
      enabled=(bool)atoi(cmd_arg);
    } else {
      Serial.print("temperature_enable="); Serial.println( enabled );
    }
  } else {
    return false;
  }
  return true;
}

void Thermistor::help() {
  Serial.println("  Thermistor");
  Serial.println("    temperature  # read the thermistor and print the result");
  Serial.println("    temperature_enable[=0,1] # enable/disable");
}

void Thermistor::write_frame_info(Print &out) {
  out.print("('temp_ntc','<f4'),('temp_counts','<i4'),");
}

void Thermistor::write_data(Print &out) {
  write_base16(out,(uint8_t*)(&reading),sizeof(reading));
  write_base16(out,(uint8_t*)(&dbridge),sizeof(dbridge));
}
