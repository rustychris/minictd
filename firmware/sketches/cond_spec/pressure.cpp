// Pressure sensor globals
#include "pressure.h"
#include "ms5803.h"

// This blocks everything if there are problems.
MS5803 sensor(ADDRESS_HIGH); // 0x76, the default for SFE board, and sensor head
double pressure_baseline;

void pressure_setup(){
  //Retrieve calibration constants for conversion math.
  // sensor.reset();
  // sensor.begin();
  
  // pressure_baseline = sensor.getPressure(ADC_4096);
}

void pressure_read() {
  float temperature_c;
  double pressure_abs, pressure_baseline;

  // To measure to higher degrees of precision use the following sensor settings:
  // ADC_256 
  // ADC_512 
  // ADC_1024
  // ADC_2048
  // ADC_4096
    
  // Read temperature from the sensor in deg C. 
  //DBG temperature_c = sensor.getTemperature(CELSIUS, ADC_512);
    
  // Read pressure from the sensor in mbar.
  //DBG pressure_abs = sensor.getPressure(ADC_4096);
     
  Serial.print("temp_ms5803==");
  Serial.print(temperature_c);
  
  Serial.print("press_abs_mbar=");
  Serial.print(pressure_abs);
     
  Serial.print("press_delta_dbar=");
  Serial.println( ( (pressure_abs-pressure_baseline)/100.0 ) ); 
}
