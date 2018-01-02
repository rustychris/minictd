/******************************************************************************
 * grabbed from github 2016-10-01
MS5803_I2C.h
Library for MS5803 pressure sensors.
Casey Kuhns @ SparkFun Electronics
6/26/2014
https://github.com/sparkfun/MS5803-14BA_Breakout

The MS58XX MS57XX and MS56XX by Measurement Specialties is a low cost I2C pressure
sensor.  This sensor can be used in weather stations and for altitude
estimations. It can also be used underwater for water depth measurements. 

In this file are the function prototypes in the MS5803 class 

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

#ifndef SparkFun_MS5803_I2C_h
#define SparkFun_MS5803_I2C_h

#include <Arduino.h>

// bastardizing Sensor to allow calls to member functions
#include "Sensor.h"

// Define units for conversions. 
enum temperature_units
{
  CELSIUS,
  FAHRENHEIT,
};

// Define measurement type.
enum measurement
{ 
  PRESSURE = 0x00,
  TEMPERATURE = 0x10
};

// Define constants for Conversion precision
enum precision
{
  ADC_256  = 0x00,
  ADC_512  = 0x02,
  ADC_1024 = 0x04,
  ADC_2048 = 0x06,
  ADC_4096 = 0x08
};

// Define address choices for the device (I2C mode)
enum ms5803_addr
{
  ADDRESS_HIGH = 0x76,
  ADDRESS_LOW  = 0x77
};

//Commands
#define CMD_RESET 0x1E // reset command 
#define CMD_ADC_READ 0x00 // ADC read command 
#define CMD_ADC_CONV 0x40 // ADC conversion command 

#define CMD_PROM 0xA0 // Coefficient location


class MS5803 : public Sensor
{
public: 
  MS5803(ms5803_addr address); 
  void reset(void);  //Reset device
  uint8_t begin(void); // Collect coefficients from sensor
  
  // Return calculated temperature from sensor
  float getTemperature(temperature_units units, precision _precision);
  // Return calculated pressure from sensor
  float getPressure(precision _precision);

  int32_t _temperature_actual; // as hundredths of deg C.
  int32_t _pressure_actual; // as tenths of Pa
  void getMeasurements(precision _precision);

  void async_getMeasurements(precision _temp_precision,precision _press_precision);
  void async_getADC_temp(void);
  void async_getADC_press(void);
  void async_conversion(uint8_t flags);
  void async_sendRead(void);
  void async_request_three(void);
  void async_readTemp(void);
  void async_readPress(void);
  void async_raw_to_actual(void);
  
  void raw_to_actual(void);
  
private:
  
  ms5803_addr _address;     // Variable used to store I2C device address.
  uint16_t coefficient[8];// Coefficients;

  precision temp_precision;
  precision press_precision;

  // the most recent measurements
  int32_t temperature_raw;
  int32_t pressure_raw;
  
  void sendCommand(uint8_t command);  // General I2C send command function
  uint32_t getADCconversion(measurement _measurement, precision _precision);  // Retrieve ADC result
  
  void sensorWait(uint8_t time); // General delay function see: delay()

};

#endif
