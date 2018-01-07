/******************************************************************************
 * Grabbed from github on 2016-10-01
MS5803_I2C.cpp
Library for MS5803 pressure sensor.
Casey Kuhns @ SparkFun Electronics
6/26/2014
https://github.com/sparkfun/MS5803-14BA_Breakout

The MS58XX MS57XX and MS56XX by Measurement Specialties is a low cost I2C pressure
sensor.  This sensor can be used in weather stations and for altitude
estimations. It can also be used underwater for water depth measurements. 

In this file are the functions in the MS5803 class

Resources:
This library uses the Arduino Wire.h to complete I2C transactions.

Development environment specifics:
  IDE: Arduino 1.0.5
  Hardware Platform: Arduino Pro 3.3V/8MHz
  MS5803 Breakout Version: 1.0
  
**Updated for Arduino 1.6.4 5/2015**

This code is beerware. If you see me (or any other SparkFun employee) at the
local pub, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#pragma GCC optimize ("O0")

#include "i2c_t3_local.h" // Wire library is used for I2C
#include "Sensor.h"
#include "ms5803.h"

// should it be volatile?
MS5803 *the_ms5803;

MS5803::MS5803(ms5803_addr address)
// Base library type I2C
{
  // Wire.begin(); // this gets called at a more controlled time in SeaDuck::setup()
  _address = address; //set interface used for communication

  the_ms5803=this;  
}

void MS5803::reset(void)
// Reset device I2C
{
   sendCommand(CMD_RESET);
   sensorWait(3);
}

uint8_t MS5803::begin(void)
// Initialize library for subsequent pressure measurements
{  
  uint8_t i;
  for(i = 0; i <= 7; i++)
  {
    sendCommand(CMD_PROM + (i * 2));
    Wire.requestFrom( _address, 2);
    uint8_t highByte = Wire.read(); 
    uint8_t lowByte = Wire.read();
    coefficient[i] = (highByte << 8)|lowByte;
    // Uncomment below for debugging output.
    //   Serial.print("C");
    //   Serial.print(i);
    //   Serial.print("= ");
    //   Serial.println(coefficient[i]);
  }

  return 0;
}
  
float MS5803::getTemperature(temperature_units units, precision _precision)
// Return a temperature reading in either F or C.
{
  getMeasurements(_precision);
  float temperature_reported;
  // If Fahrenheit is selected return the temperature converted to F
  if(units == FAHRENHEIT)
  {
    temperature_reported = _temperature_actual / 100.0f;
    temperature_reported = (((temperature_reported) * 9) / 5) + 32;
    return temperature_reported;
  }
    
  // If Celsius is selected return the temperature converted to C 
  else
  {
    temperature_reported = _temperature_actual / 100.0f;
    return temperature_reported;
  }
}

float MS5803::getPressure(precision _precision)
// Return a pressure reading units Pa.
{
  getMeasurements(_precision);
  float pressure_reported;
  pressure_reported = _pressure_actual;
  pressure_reported = pressure_reported / 10.0f;
  return pressure_reported;
}

void MS5803::getMeasurements(precision _precision)
{
  //Retrieve ADC result
  temperature_raw = getADCconversion(TEMPERATURE, _precision);
  pressure_raw = getADCconversion(PRESSURE, _precision);
  
  raw_to_actual();
}

// Assumes that temperature_raw and pressure_raw have been
// read in.  Applies calibrations to populate _temperature_actual
// and _pressure_actual
void MS5803::raw_to_actual(void)
{
  //Create Variables for calculations
  int32_t temp_calc;
  int32_t pressure_calc;
  
  int32_t dT;
    
  //Now that we have a raw temperature, let's compute our actual.
  dT = temperature_raw - ((int32_t)coefficient[5] << 8);
  temp_calc = (((int64_t)dT * coefficient[6]) >> 23) + 2000;
  
  // TODO TESTING  _temperature_actual = temp_calc;
  
  //Now we have our first order Temperature, let's calculate the second order.
  int64_t T2, OFF2, SENS2, OFF, SENS; //working variables

  if (temp_calc < 2000) 
  // If temp_calc is below 20.0C
  { 
    T2 = 3 * (((int64_t)dT * dT) >> 33);
    OFF2 = 3 * ((temp_calc - 2000) * (temp_calc - 2000)) / 2;
    SENS2 = 5 * ((temp_calc - 2000) * (temp_calc - 2000)) / 8;
    
    if(temp_calc < -1500)
    // If temp_calc is below -15.0C 
    {
      OFF2 = OFF2 + 7 * ((temp_calc + 1500) * (temp_calc + 1500));
      SENS2 = SENS2 + 4 * ((temp_calc + 1500) * (temp_calc + 1500));
    }
    } 
  else
  // If temp_calc is above 20.0C
  { 
    T2 = 7 * ((uint64_t)dT * dT)/pow(2,37);
    OFF2 = ((temp_calc - 2000) * (temp_calc - 2000)) / 16;
    SENS2 = 0;
  }
  
  // Now bring it all together to apply offsets 
  
  OFF = ((int64_t)coefficient[2] << 16) + (((coefficient[4] * (int64_t)dT)) >> 7);
  SENS = ((int64_t)coefficient[1] << 15) + (((coefficient[3] * (int64_t)dT)) >> 8);
  
  temp_calc = temp_calc - T2;
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;

  // Now lets calculate the pressure
  

  pressure_calc = (((SENS * pressure_raw) / 2097152 ) - OFF) / 32768;
  
  _temperature_actual = temp_calc ;
  _pressure_actual = pressure_calc ; // 10;// pressure_calc;
  
}

// this gets called once the conversion request has been
// sent.

void MS5803::async_sendRead() {
  Wire.beginTransmission( _address );
  Wire.write(CMD_ADC_READ);
  Wire.onTransmitDone(pop_fn_and_call); 
  Wire.sendTransmission();
}

void MS5803::async_getADC_temp()
{
  push_fn(this,(SensorFn)&MS5803::async_readTemp);
  push_fn(this,(SensorFn)&MS5803::async_request_three);
  push_fn(this,(SensorFn)&MS5803::async_sendRead);

  async_conversion(TEMPERATURE+temp_precision);
}

// Starts the chain of functions to ultimately get a new
// temp/pressure reading
void MS5803::async_getMeasurements(precision _temp_precision,precision _press_precision)
{
  temp_precision=_temp_precision;  // ...
  press_precision=_press_precision; // these may move...

  // it's a stack, so LIFO
  push_fn(this,(SensorFn)&MS5803::async_raw_to_actual);
  push_fn(this,(SensorFn)&MS5803::async_getADC_temp);
  push_fn(this,(SensorFn)&MS5803::async_getADC_press);
  // set this in motion
  pop_fn_and_call();
}

void MS5803::async_getADC_press(void)
{
  push_fn(this,(SensorFn)&MS5803::async_readPress);
  push_fn(this,(SensorFn)&MS5803::async_request_three);
  push_fn(this,(SensorFn)&MS5803::async_sendRead);

  async_conversion(PRESSURE+press_precision);
}

void end_delay_and_pop() {
  sensorTimer.end();
  
  pop_fn_and_call(); 
}

// start the conversion process
void MS5803::async_conversion(uint8_t flags)
{
  // flags is measurement + precision
  Wire.beginTransmission( _address);
  Wire.write(CMD_ADC_CONV + flags);

  // HERE - technically we should wait until the transmit completes,
  // and then wait for 11ms.  But it's the 11ms that really holds things
  // up, so this should be a timed interrupt call instead of async i2c.
  // Wire.onTransmitDone(pop_fn_and_call);
  uint32_t delay_ms=millis_for_flags(flags);
    
  if ( ! sensorTimer.begin(end_delay_and_pop,1000*delay_ms) ) {
    Serial.println("No timers available!");
  }
  // make sure that we do not trigger an ISR on transmit done
  Wire.onTransmitDone(NULL);
  Wire.sendTransmission();
}

void MS5803::async_request_three(void)
{
  Wire.onReqFromDone(pop_fn_and_call);
  Wire.sendRequest(_address, 3);
}

void MS5803::async_readTemp(void)
{
  uint8_t highByte = 0, midByte = 0, lowByte = 0;

  while(Wire.available()) // RH: not sure why that's a loop..   
  { 
    highByte = Wire.read();
    midByte = Wire.read();
    lowByte = Wire.read();  
  }
  
  temperature_raw=((uint32_t)highByte << 16) + ((uint32_t)midByte << 8) + lowByte;

  pop_fn_and_call();
}


void MS5803::async_readPress(void)
{
  uint8_t highByte = 0, midByte = 0, lowByte = 0;
  while(Wire.available()) // RH: not sure why that's a loop..   
  {
    highByte = Wire.read();
    midByte = Wire.read();
    lowByte = Wire.read();  
  }
  
  pressure_raw=((uint32_t)highByte << 16) + ((uint32_t)midByte << 8) + lowByte;

  pop_fn_and_call();
}

void MS5803::async_raw_to_actual(void)
{
  Wire.onTransmitDone(NULL);
  Wire.onReqFromDone(NULL);
  
  raw_to_actual();

  pop_fn_and_call();
}

uint32_t millis_for_flags(uint8_t flags) {
  switch( flags & ADC_MASK )
    {
      // these include the 1ms "general delay"
    case ADC_256 : return 2;
    case ADC_512 : return 4; 
    case ADC_1024: return 5; 
    case ADC_2048: return 7; 
    case ADC_4096: return 11;
    }
  return 11; // shouldn't happen
}

uint32_t MS5803::getADCconversion(measurement _measurement, precision _precision)
// Retrieve ADC measurement from the device.  
// Select measurement type and precision
{ 
  uint32_t result;
  uint8_t highByte = 0, midByte = 0, lowByte = 0;

  uint8_t flags=CMD_ADC_CONV + _measurement + _precision;
  sendCommand(flags);
  sensorWait(millis_for_flags(flags));
  
  sendCommand(CMD_ADC_READ);
  Wire.requestFrom(_address, 3);
  
  while(Wire.available())    
  { 
    highByte = Wire.read();
    midByte = Wire.read();
    lowByte = Wire.read();  
  }
  
  result = ((uint32_t)highByte << 16) + ((uint32_t)midByte << 8) + lowByte;

  return result;

}

void MS5803::sendCommand(uint8_t command)
{ 
  Wire.beginTransmission( _address);
  Wire.write(command);
  Wire.endTransmission();
  
}

void MS5803::sensorWait(uint8_t time)
// Delay function.  This can be modified to work outside of Arduino based MCU's
{
  delay(time);
}

