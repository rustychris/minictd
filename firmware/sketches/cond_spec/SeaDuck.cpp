#include "cfg_seaduck.h"

#include <AWire.h>
#include <elapsedMillis.h>

#include "serialmux.h"

#include "SeaDuck.h"
#include "SdFunctions.h"
#include "Sensor.h"

#ifdef USE_TEENSY_ADC
ADC *adc=new ADC();
#endif

IntervalTimer Timer(3);

#ifdef HAS_PRESSURE
#include "pressure.h"
Pressure pressure;
#endif

#ifdef HAS_CONDUCTIVITY
#include "conductivity.h"
Conductivity cond;
#endif

#ifdef HAS_NTC
#include "thermistor.h"
Thermistor ntc;
#endif

#ifdef HAS_RTC_DS3231
#include "rtc_ds3231.h"
RTC_DS3231 clock;
#endif

#ifdef HAS_IMU
#include "imu.h"
IMU imu0;
#endif

#ifdef HAS_GPS
#include "gps.h"
GPS gps;
#endif

#ifdef HAS_MOTOR
#include "motor.h"
Motor motor;
#endif

#ifdef HAS_BUOYANCY
#include "buoyancy.h"
Buoyancy buoyancy;
#endif

#ifdef DOTSTAR_CLK
#include <Adafruit_DotStar.h>
Adafruit_DotStar dotstar = Adafruit_DotStar(1,DOTSTAR_DATA,DOTSTAR_CLK);  
#endif

Storage storage;

#ifdef USE_TEENSY_ADC
void check_adc_error() {
  if ( adc->adc1->fail_flag != ADC_ERROR_CLEAR ) {
      mySerial.print(" ADC1 error: ");
      mySerial.println(adc->adc1->fail_flag);
      adc->adc1->fail_flag=ADC_ERROR_CLEAR;
      while(1);
  }
  if ( adc->adc0->fail_flag != ADC_ERROR_CLEAR ) {
      mySerial.print(" ADC0 error: ");
      mySerial.println(adc->adc0->fail_flag);
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

#ifdef HAS_IMU
  sensors[num_sensors++]=&imu0;
#endif

#ifdef HAS_PRESSURE
  sensors[num_sensors++]=&pressure;
#endif // HAS_PRESSURE
#ifdef HAS_CONDUCTIVITY
  sensors[num_sensors++]=&cond;
#endif
#ifdef HAS_NTC
  sensors[num_sensors++]=&ntc;
#endif
#ifdef HAS_RTCLOCK
  sensors[num_sensors++]=&clock;
#endif

#ifdef HAS_RTC_DS3231
  sensors[num_sensors++]=&clock;
#endif

#ifdef HAS_GPS
  sensors[num_sensors++]=&gps;
#endif

#ifdef HAS_MOTOR
  sensors[num_sensors++]=&motor;
#endif
  
#ifdef HAS_BUOYANCY
  // Should be *after* pressure and motor
  sensors[num_sensors++]=&buoyancy;
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
  AWire.begin(); 
#endif
  
  mySerial.println("# SeaDuck setup");

#ifdef USE_TEENSY_ADC
  common_adc_init();
#endif

#ifdef STATUS_LED
  pinMode(STATUS_LED,OUTPUT); // green led
#endif

  for(int i=0;i<num_sensors;i++){
    mySerial.print("# ");
    mySerial.print(sensors[i]->name);
    sensors[i]->init();
    mySerial.println("   done");
  }

  binary_format=BIN_HEX;
  storage.init();
  mySerial.println("# Storage init");

#ifdef DOTSTAR_CLK
  // this had been working, but after accidentally flashing it as a
  // feather, then switching back to itsy bitsy m0, I can no longer turn
  // the dotstar off?
  dotstar.begin();
  dotstar.clear(); 
  dotstar.show();// turns off?
#endif
  
  mySerial.println("# Checking for " CMDFILE);
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
    mySerial.print("interval_us="); mySerial.println(sample_interval_us);
#ifdef BT2S
  } else if ( strcmp("bt_passthrough",cmd)==0 ) {
    mySerial.pass_through();
  } else if ( strcmp("bt_baud",cmd)==0 ) {
    if(cmd_arg) {
      mySerial.bt_baud=atoi(cmd_arg);
    }
    mySerial.print("bt_baud="); mySerial.println(mySerial.bt_baud);
    mySerial.println("# Will not take effect until after a bt_passthrough");
#endif
  } else {
    Shell::dispatch_command();
  }
}

void SeaDuck::help() {
  Shell::help();
  mySerial.println("    sample # one-shot sampling");
  mySerial.println("    sample_loop # continuous sampling");
  mySerial.println("    interval_us[=NNNN] # sampling interval in usecs");
  for(int i=0;i<num_sensors;i++ ) {
    sensors[i]->help();
  }
  storage.help();
}

time_t SeaDuck::unixtime() {
#ifdef HAS_RTC_DS3231
  clock.read();
  return clock.reading_seconds;
#else
  // Punt with time since startup.  Maybe at some point
  // can get a GPS based time in the absence of RTC.
  return (time_t)(millis()/1000);
#endif
}

void SeaDuck::write_header(void) {
  Print *out=&mySerial;
  
  if( storage.status==Storage::ENABLED ) {
    out=&storage;
  }
  out->print("[");
  for(int i=0;i<num_sensors;i++ ) {
    if ( sensors[i]->enabled ) {
      sensors[i]->write_frame_info(*out);
    }
  }
  out->println("]");
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
  Print *out=&mySerial;
  
  if( storage.status==Storage::ENABLED ) {
    out=&storage;
  }
  
  for(int i=0;i<num_sensors;i++ ) {
    if ( sensors[i]->enabled ) {
      sensors[i]->write_data(*out);
    }
  }
  // probably this moves elsewhere to keep output from continuous sampling cleaner.
  // doesn't really seem necessary
  // out->print("STOP\n");
  // but newlines are nice
  out->print("\n");
  out->flush(); // BT Serial.flush() hangs, so that's disabled.
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
    mySerial.println("Skipping timed sample because the stack is not empty");
  } else {
    logger.async_oneshot();
  }
}

// asynchronous repeated sampling
void SeaDuck::continuous_sample(void) {
  elapsedMillis elapsed;

  // start with data definition
  write_header();

  for(int i=0;i<num_sensors;i++ ) {
    if ( sensors[i]->enabled ) {
      sensors[i]->enter_sample_loop();
    }
  }
  
  mySerial.println("# Starting interval timer loop");
  Timer.begin(timer_isr,sample_interval_us);

  while ( 1 ) {
#ifdef STATUS_LED
    if(elapsed<30*1000) {
      digitalWrite(STATUS_LED,HIGH);
    } else {
      digitalWrite(STATUS_LED,LOW);
    }          
#endif
    storage.loop();

    // Allow stopping the loop on ! or ESC
    // Serial.println("About to check for available input");
    if( mySerial.available() ) {
      // Serial.println("About to read available character");
      uint8_t c=mySerial.read();
      // stop it when an exclamation or ESC is read
      if ( (c=='!') || (c==27) ) {
        break;
      } 
    }
  }
  Timer.end();
  while( stack_size() ) ; // let any currently running sampling code finish

  for(int i=0;i<num_sensors;i++ ) {
    if ( sensors[i]->enabled ) {
      sensors[i]->exit_sample_loop();
    }
  }

  // loop may have been leaving some data in buffers.
  // I think cleanup is the correct one.
  // storage.loop(); 
  storage.cleanup();

#ifdef STATUS_LED
  digitalWrite(STATUS_LED,LOW);
#endif

  mySerial.println("# Exiting interval timer loop. ");
}
