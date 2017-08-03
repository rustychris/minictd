//-------------------Thermistor-------------//
#include "seaduck_cfg.h"
#include "SeaDuck.h"

void ntc_setup() {
  pinMode(NTC_SENSE,INPUT);
}

void ntc_read() {
  int dbridge;
  float dratio,temp;
  float R,T;

  // HERE -- use direct acd programming, or at least compatible with
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
  // currently, breathing on it makes the numbers go more negative.
  Serial.print("[Temp] bridge counts: ");
  Serial.print(dbridge);
  Serial.print("  mV: ");
  Serial.print( 3.3*1000*dratio );
  Serial.print("  R(ohm): ");
  Serial.print(R);

  //fiction!
  T=23.0 - (R-108000)*0.0005;
  Serial.print(" T(degC): ");
  Serial.print(T);
  
  Serial.println("");
}
