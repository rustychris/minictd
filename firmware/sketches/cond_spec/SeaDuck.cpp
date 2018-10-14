#ifdef CORE_TEENSY
#include "i2c_t3_local.h"
#else
#include <Wire.h>
#endif


#include "SeaDuck.h"
#include "pressure.h"
#include "thermistor.h"
#include "conductivity.h"
#include "rtclock.h"
#include "SdFunctions.h"
#include "Sensor.h"

#ifdef USE_TEENSY_ADC
ADC *adc=new ADC();
#endif

IntervalTimer Timer;

Pressure pressure;

#ifdef HAS_CONDUCTIVITY
Conductivity cond;
#endif
#ifdef HAS_NTC
Thermistor ntc;
#endif
#ifdef HAS_RTCLOCK
RTClock clock;
#endif

Storage storage;

#ifdef USE_TEENSY_ADC
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

#endif // USE_ADC


SeaDuck::SeaDuck() 
{
  sample_interval_us=100000; // 10 Hz

  num_sensors=0;
  sensors[num_sensors++]=&pressure;
#ifdef HAS_CONDUCTIVITY
  sensors[num_sensors++]=&cond;
#endif
#ifdef HAS_NTC
  sensors[num_sensors++]=&ntc;
#endif
#ifdef HAS_RTCLOCK
  sensors[num_sensors++]=&clock;
#endif

#ifdef POWER_3V3_ENABLE_PIN
  pinMode(POWER_3V3_ENABLE_PIN,OUTPUT);
  digitalWrite(POWER_3V3_ENABLE_PIN,HIGH);
#endif
}
 
void SeaDuck::setup() {
  Shell::setup();

#ifdef CORE_TEENSY
  // have to select non-default pins
  Wire.begin(I2C_MASTER, // mode
             0, // address - ignored
             I2C_PINS_16_17, // match with schematic
             I2C_PULLUP_EXT, // we have external pullups
             400000 // rate is 400kHz
             );
#else
  Wire.begin();
#endif
  
  Serial.println("# SeaDuck setup");

#ifdef USE_TEENSY_ADC
  common_adc_init();
#endif

  binary_format=BIN_HEX;
  
  storage.init();

  Serial.println("# Storage init");
  
  for(int i=0;i<num_sensors;i++){
    Serial.print("# ");
    Serial.print(sensors[i]->name);
    sensors[i]->init();
    Serial.println("   done");
  }

  Serial.println("# Checking for " CMDFILE);
  activate_cmd_file(CMDFILE);
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
  Serial.println("    sample_loop # continuous sampling");
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

void SeaDuck::write_header(void) {
  Serial.print("[");
  for(int i=0;i<num_sensors;i++ ) {
    if ( sensors[i]->enabled ) {
      sensors[i]->write_frame_info(Serial);
    }
  }
  Serial.println("]");
}

// header is written sync, and sampling and data output are async.
void SeaDuck::oneshot_sample(void) {
  write_header();

  sensors[0]->push_busy(); // hack - borrow somebody's busy. maybe it shouldn't be in Sensors?

  async_oneshot();
    
  while(sensors[0]->busy) {
    storage.loop();
  }
  storage.loop();
}

void SeaDuck::async_output(void) {
  Print *out=&Serial;
  
  if( storage.status==Storage::ENABLED ) {
    out=&storage;
  }
  
  for(int i=0;i<num_sensors;i++ ) {
    if ( sensors[i]->enabled ) {
      sensors[i]->write_data(*out);
    }
  }
  // probably this moves elsewhere to keep output from continuous sampling cleaner.
  out->print("STOP\n"); 
  out->flush();
  pop_fn_and_call();
}

// async one-shot
void SeaDuck::async_oneshot(void) {
  // in this case, we don't output the header, since this fn will be
  // the basis of async_continuous, and we only need the header once.
  // This pointer might be a problem since SeaDuck is not a subclass of Sensor
  push_fn((Sensor*)this,(SensorFn)&SeaDuck::async_output);
  
  for(int i=0;i<num_sensors;i++ ) {
    if ( sensors[i]->enabled ) {
      push_fn(sensors[i],(SensorFn)&Sensor::async_read);
    }
  }
  pop_fn_and_call();  
}

void timer_isr(void) {
  if ( stack_size() > 0 ) {
    Serial.println("Skipping timed sample because the stack is not empty");
  } else {
    logger.async_oneshot();
  }
}

// asynchronous repeated sampling
void SeaDuck::continuous_sample(void) {
  unsigned long t_start=millis();

  Serial.println("# Starting interval timer loop");
  Timer.begin(timer_isr,sample_interval_us);

  // This is just for testing anyway -- loop for a limited amount
  // of time.
  while ( millis()-t_start < 2000 ) {
    storage.loop();

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
  while( stack_size() ) ; // let any currently running sampling code finish
  storage.loop();
  
  Serial.println("# Exiting interval timer loop. ");
}
