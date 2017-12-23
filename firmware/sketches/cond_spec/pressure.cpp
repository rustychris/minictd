// Pressure sensor globals
#include "pressure.h"
#include "ms5803.h"

// This blocks everything if there are problems.
MS5803 sensor(ADDRESS_HIGH); // 0x76, the default for SFE board, and sensor head
// MS5803 sensor(ADDRESS_LOW); // 0x76, the default for SFE board, and sensor head

void Pressure::init(){
  //Retrieve calibration constants for conversion math.
  sensor.reset();
  sensor.begin();

  read();
  pressure_baseline_dPa = pressure_abs_dPa;
}

void Pressure::read() {
  // To measure to higher degrees of precision use the following sensor settings:
  // ADC_256 
  // ADC_512 
  // ADC_1024
  // ADC_2048
  // ADC_4096
    
  // Read temperature from the sensor in deg C.
  sensor.getMeasurements(ADC_4096);

  temperature_c100 = sensor._temperature_actual;

  // 1 bar ~ 100kPa
  // Read pressure from the sensor in mbar.  or is it tenths of Pa?
  pressure_abs_dPa = sensor._pressure_actual;
}

void Pressure::display(){
  read();
  
  Serial.print("temp_ms5803==");
  Serial.println(temperature_c100 / 100.0f);
  
  Serial.print("press_abs_mbar=");
  Serial.println(pressure_abs_dPa / 10.0f);
     
  Serial.print("press_delta_dbar=");
  Serial.println( ( (pressure_abs_dPa-pressure_baseline_dPa)/1000.0 ) ); 
}

bool Pressure::dispatch_command(const char *cmd, const char *cmd_arg) {
  if ( !strcmp(cmd,"pressure") ) {
    display();
  } else {
    return false;
  }
  return true;
}

void Pressure::help() {
  Serial.println("  Pressure");
  Serial.println("    pressure  # report temp and pressure");
}

  
void Pressure::write_frame_info(Print &out) {
  out.print( "('pressure_dPa','<i4'),('temp_c100_ms5803','<i4'),");
}

void Pressure::write_data(Print &out){
  write_base16(out,(uint8_t*)&pressure_abs_dPa,sizeof(pressure_abs_dPa));
  write_base16(out,(uint8_t*)&temperature_c100,sizeof(temperature_c100));
}

