/* 
 *  Explore a wider variety of how to run the cell, basically
 *  trying some spectroscopy
 */
#include "cfg_seaduck.h"
#include <SPI.h>
#include <SdFat.h>

#include <AWire.h>

#include <TimeLib.h>

#ifdef HAS_CONDUCTIVITY
#include <ADC.h>
#include <DMAChannel.h>
#include "conductivity.h"
#endif

#include "SeaDuck.h"
#include "ms5803.h"

#ifdef HAS_NTC
#include "thermistor.h"
#endif

#ifdef HAS_IMU
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "imu.h"
#endif

#ifdef HAS_RTC_DS3231
#include "rtc_ds3231.h"
#endif

#ifdef HAS_MOTOR
#include "motor.h"
#endif

#ifdef HAS_BUOYANCY
#include "buoyancy.h"
#endif

#ifdef DOTSTAR_CLK
#include <Adafruit_DotStar.h>
#endif

#include "SdFunctions.h"

SeaDuck logger;

void setup() {
  logger.setup();
}

void loop() {
  logger.loop();
}
