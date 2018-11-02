/* 
 *  Explore a wider variety of how to run the cell, basically
 *  trying some spectroscopy
 */
#include "cfg_seaduck.h"
#include <SPI.h>
#include <SdFat.h>

#ifdef CORE_TEENSY
#include "i2c_t3_local.h"
#endif

#include <TimeLib.h>

#ifdef HAS_CONDUCTIVITY
#include <ADC.h>
#include <DMAChannel.h>
#include "conductivity.h"
#endif

#include "SeaDuck.h"
#include "ms5803.h"

#ifndef CORE_TEENSY
// SAMD async I2C
#include "I2C_DMAC.h"
#include "DMAC.h"
#endif

#ifdef HAS_NTC
#include "thermistor.h"
#endif

#ifdef HAS_IMU
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#endif

#include "rtclock.h"
#include "SdFunctions.h"

SeaDuck logger;

void setup() {
  logger.setup();
}

void loop() {
  logger.loop();
}
