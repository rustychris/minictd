#include "i2c_t3_local.h"

#include "SeaDuck.h"
#include "pressure.h"
#include "thermistor.h"
#include "conductivity.h"
#include "rtclock.h"
#include "SdFunctions.h"
#include "Sensor.h"

ADC *adc=new ADC();

IntervalTimer Timer;

Pressure pressure;
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
  sample_interval_us=50000; // 20 Hz

  num_sensors=4;
  sensors[0]=&pressure;
  sensors[1]=&cond;
  sensors[2]=&ntc;
  sensors[3]=&clock;
  
  pinMode(POWER_3V3_ENABLE_PIN,OUTPUT);
  digitalWrite(POWER_3V3_ENABLE_PIN,HIGH);
}

 
void SeaDuck::setup() {
  Shell::setup();

  // have to select non-default pins
  Wire.begin(I2C_MASTER, // mode
             0, // address - ignored
             I2C_PINS_16_17, // match with schematic
             I2C_PULLUP_EXT, // we have external pullups
             400000 // rate is 400kHz
             );
  
  Serial.println("SeaDuck setup");
  common_adc_init();

  binary_format=BIN_HEX;
  
  storage.init();

  Serial.println("Storage init");
  
  for(int i=0;i<num_sensors;i++){
    Serial.println(i);
    sensors[i]->init();
  }
}

void SeaDuck::dispatch_command() {
  for(int i=0;i<num_sensors;i++) {
    if( sensors[i]->dispatch_command(cmd,cmd_arg) )
      return;
  }
  if ( storage.dispatch_command(cmd,cmd_arg) ) {
    return;
  } else if ( strcmp("sample",cmd)==0 ) {
    oneshot_sample();
  } else if ( strcmp("sample_loop",cmd)==0 ) {
    continuous_sample();
  } else if ( strcmp("interval_us",cmd)==0 ) {
    if(cmd_arg) {
      sample_interval_us=atoi(cmd_arg);      
    }
    Serial.print("interval_us="); Serial.println(sample_interval_us);
  } else {
    Shell::dispatch_command();
  }
}

void SeaDuck::help() {
  Shell::help();
  Serial.println("    sample # one-shot sampling");
  Serial.println("    interval_us[=NNNN] # sampling interval in usecs");
  for(int i=0;i<num_sensors;i++ ) {
    sensors[i]->help();
  }
  storage.help();
}

time_t SeaDuck::unixtime() {
  clock.read();
  return clock.reading_seconds;
}

void SeaDuck::oneshot_sample(void) {
  
  Serial.print("[");
  for(int i=0;i<num_sensors;i++ ) {
    if ( sensors[i]->enabled ) {
      sensors[i]->write_frame_info(Serial);
    }
  }
  Serial.println("]");
  
  for(int i=0;i<num_sensors;i++ ) {
    if ( sensors[i]->enabled ) {
      sensors[i]->read();
      sensors[i]->write_data(Serial);
    }
  }
  Serial.print("STOP\n"); 
  Serial.flush(); // having trouble seeing any of that stuff above.
}

volatile int sample_flag=0;

void timer_isr(void) {
  cli();
  sample_flag++;
  sei();
}

void SeaDuck::continuous_sample(void) {
  // Development:
  // 3. Fire sampling from this interrupt
  // 4. Get on to recording to SD

  // best approach is to use another timer, like IntervalTimer, and
  //   read the RTC to get timestamps.

  // for starters, go for some set number of seconds
  time_t t_start=Teensy3Clock.get();
  long start,loop_time;

  Serial.println("# Starting interval timer loop");
  Timer.begin(timer_isr,sample_interval_us);

  int my_sample=sample_flag;
  
  while ( Teensy3Clock.get() < t_start+2 ) {
    // spin wait for next sample to get triggered:
    // Should check to see if this is actually safe.
    while( my_sample == sample_flag ) ;
    start=millis();
    my_sample++;
    
    oneshot_sample();

    loop_time=millis() - start;

    // Allow stopping the loop on ! or ESC
    if( Serial.available() ) {
      uint8_t c=Serial.read();
      // stop it when an exclamation or ESC is read
      if ( (c=='!') || (c==27) ) {
        break;
      } 
    }
  }
  Timer.end();
  Serial.println("# Exiting interval timer loop. ");
  Serial.print(loop_time);
  Serial.println(" ms for last loop");
}
