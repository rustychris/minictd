#include <Wire.h>

#include "SeaDuck.h"
#include "pressure.h"
#include "thermistor.h"
#include "conductivity.h"
#include "rtclock.h"
#include "SdFunctions.h"

ADC *adc=new ADC();

Conductivity cond;
Thermistor ntc;
RTClock clock;
Storage storage;

void check_adc_error() {
  if ( adc->adc1->fail_flag != ADC_ERROR_CLEAR ) {
      Serial.print(" ADC1 error: ");
      Serial.println(adc->adc1->fail_flag);
      adc->adc1->fail_flag=ADC_ERROR_CLEAR;
      while(1);
  }
  if ( adc->adc0->fail_flag != ADC_ERROR_CLEAR ) {
      Serial.print(" ADC0 error: ");
      Serial.println(adc->adc0->fail_flag);
      adc->adc0->fail_flag=ADC_ERROR_CLEAR;
      while(1);
  }
}

/*
 * Configure aspects of the ADCs that will not change
 * with whether it's being used by conductivity or
 * thermistor
 */
void common_adc_init(void) {
  for(int adc_num=0;adc_num<2;adc_num++) {
    // used to be ADC_REF_3V3
    adc->setReference(ADC_REFERENCE::REF_3V3, adc_num);
    delay(5);
    adc->setResolution(16,adc_num);
    delay(5);
    adc->setAveraging(1,adc_num);
    adc->disablePGA(adc_num);
    delay(5);

    // default settings are good down to pdb period of 300.
    if(0) {
      // these settings are okay down to pdb period of 175.
      // adc->setConversionSpeed(ADC_HIGH_SPEED_16BITS,adc_num);
      adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS,adc_num);
      delay(5);
      adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED,adc_num);
      delay(5);
    } else {
      // probably ill-advised
      adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED,adc_num);
      delay(5);
      adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED,adc_num);
      delay(5);
    }
  }
  
  check_adc_error();
  // just to be sure that ADC trigger is from PDB.  This configures the source
  // of the trigger, but whether the ADC is automatically triggered or not
  // is configured elsewhere, so this is safe even if direct triggering for
  // the thermistor is being used
  SIM_SOPT7=0;
}


SeaDuck::SeaDuck() 
{
  pinMode(POWER_3V3_ENABLE_PIN,OUTPUT);
  digitalWrite(POWER_3V3_ENABLE_PIN,HIGH);
}


 
void SeaDuck::setup() {
  Shell::setup();

  Wire.begin();
  
  Serial.println("SeaDuck setup");
  common_adc_init();

  cond.init();
  pressure_setup();
  ntc.init();
  clock.init();
}

void SeaDuck::dispatch_command() {
  if ( cond.dispatch_command(cmd,cmd_arg) ) {
    return;
  } else if ( strcmp(cmd,"pressure")==0 ) {
    pressure_read();
  } else if ( ntc.dispatch_command(cmd,cmd_arg) ) {
    return;
  } else if ( clock.dispatch_command(cmd,cmd_arg) ) {
    return;
  } else {
    Shell::dispatch_command();
  }
}

void SeaDuck::help() {
  Shell::help();
  cond.help();
  ntc.help();
  clock.help();
  storage.help();
}



