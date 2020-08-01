/*************************************************** 
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(6);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0

unsigned long t_last;

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");

  thermo.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary
  thermo.readRTD(); // gets it into a good state
  thermo.autoConvert(true);
  t_last=millis();
}


void loop() {
  unsigned long t_new=millis();
  delay(25);
  uint16_t rtd = thermo.readRTD_continuous();

  // Serial.print("RTD value: "); 
  Serial.print(rtd);
  // Serial.print("  elapsed_ms: "); Serial.print( t_new-t_last );
  t_last=t_new;
  // This stuff ends up being kind of slow.
  // Max conversion time is 21ms.  This loop currently is at 76ms.
  // float ratio = rtd;
  // ratio /= 32768;
  // Serial.print("  Ratio = "); Serial.print(ratio,8);
  // Serial.print("  Resistance = "); Serial.print(RREF*ratio,8);
  // Serial.print("  Temperature = "); Serial.print(thermo.temperature(RNOMINAL, RREF));

  // Check and print any faults
  if ( 0 ) {
    uint8_t fault = thermo.readFault();
    if (fault) {
      Serial.print("Fault 0x"); Serial.println(fault, HEX);
      if (fault & MAX31865_FAULT_HIGHTHRESH) {
        Serial.println("RTD High Threshold"); 
      }
      if (fault & MAX31865_FAULT_LOWTHRESH) {
        Serial.println("RTD Low Threshold"); 
      }
      if (fault & MAX31865_FAULT_REFINLOW) {
        Serial.println("REFIN- > 0.85 x Bias"); 
      }
      if (fault & MAX31865_FAULT_REFINHIGH) {
        Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_RTDINLOW) {
        Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_OVUV) {
        Serial.println("Under/Over voltage"); 
      }
      thermo.clearFault();
    }
  }
  Serial.println();
}
