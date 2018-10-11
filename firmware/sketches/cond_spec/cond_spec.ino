/* 
 *  Explore a wider variety of how to run the cell, basically
 *  trying some spectroscopy
 */
#include "cfg_seaduck.h"
#include <SPI.h>
#include <SdFat.h>

#include "i2c_t3_local.h"
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

#include "rtclock.h"
#include "SdFunctions.h"

SeaDuck logger;

void setup() {
  logger.setup();
}

void loop() {
  logger.loop();
}
