//-------------------Thermistor-------------//
#include "seaduck_cfg.h"
#include "SeaDuck.h"
#include "thermistor.h"

void Thermistor::init() {
  pinMode(NTC_SENSE,INPUT);
}

void Thermistor::read() {
  int dbridge;
  float dratio;
  float R;

  // use direct adc programming, or at least compatible with
  // SeaDuck.cpp
  adc->setAveraging(32,NTC_ADC);

  check_adc_error();

  dbridge=adc->analogRead(NTC_SENSE,NTC_ADC);

  check_adc_error();

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
  
  // Serial.print("# [Temp] bridge counts: ");
  // Serial.print(dbridge);
  // Serial.print("  mV: ");
  // Serial.print( 3.3*1000*dratio );
  // Serial.print("  R(ohm): ");
  // Serial.print(R);

  //fiction!
  reading=23.0 - (R-108000)*0.0005;
  // Serial.print(" T(degC): ");
  // Serial.print(reading);
  // 
  // Serial.println("");
}

bool Thermistor::dispatch_command(const char *cmd, const char *cmd_arg) {
  if( strcmp(cmd,"temperature")==0 ) {
    read();
    Serial.print("temperature=");
    Serial.println(reading);
  } else {
    return false;
  }
  return true;
}

void Thermistor::help() {
  Serial.println("  Thermistor");
  Serial.println("    temperature  # read the thermistor and print the result");
}

void Thermistor::write_frame_info(Print &out) {
  out.print("('temp_ntc','<f4')");
}

void Thermistor::write_data(Print &out) {
  write_base16(out,(uint8_t*)(&reading),sizeof(reading));
}
