// Pressure sensor globals
#include "cfg_seaduck.h"
#ifdef HAS_PRESSURE

#include "serialmux.h"
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
  
  pressure_baseline_dPa = sensor._pressure_actual; // pressure_abs_dPa;
}

void Pressure::async_read() {
  // set to known values to know if they actually ran.
  sensor.temperature_raw=37;
  sensor.pressure_raw=37;

  push_fn(this,(SensorFn)&Pressure::async_copyValues);
  
  // first is temperature precision, second is pressure precision
  // sensor.async_getMeasurements(ADC_4096,ADC_4096);
  sensor.async_getMeasurements(ADC_1024,ADC_1024);
}

void Pressure::async_copyValues() {
  pressure_abs_dPa=sensor._pressure_actual;
  temperature_c100=sensor._temperature_actual;
  pop_fn_and_call();
}

void Pressure::display(){
  read();
  
  mySerial.print("temp_ms5803=");
  mySerial.println(temperature_c100 / 100.0f);
  
  mySerial.print("press_abs_mbar=");
  mySerial.println(pressure_abs_dPa / 10.0f);
     
  mySerial.print("press_delta_dbar=");
  mySerial.println( ( (pressure_abs_dPa-pressure_baseline_dPa)/1000.0f ) ); 
}

bool Pressure::dispatch_command(const char *cmd, const char *cmd_arg) {
  if ( !strcmp(cmd,"pressure") ) {
    display();
  } else if ( strcmp(cmd,"pressure_enable")==0 ) {
    if(cmd_arg) {
      enabled=(bool)atoi(cmd_arg);
    } else {
      mySerial.print("pressure_enable="); mySerial.println( enabled );
    }
  } else {
    return false;
  }
  return true;
}

void Pressure::help() {
  mySerial.println("  Pressure");
  mySerial.println("    pressure  # report temp and pressure");
  mySerial.println("    pressure_enable[=0,1] # enable/disable ");
}

  
void Pressure::write_frame_info(Print &out) {
  out.print( "('pressure_dPa','<i4'),('temp_c100_ms5803','<i4'),");
}

void Pressure::write_data(Print &out){
  write_base16(out,(uint8_t*)&pressure_abs_dPa,sizeof(pressure_abs_dPa));
  write_base16(out,(uint8_t*)&temperature_c100,sizeof(temperature_c100));
}

#endif // HAS_PRESSURE
