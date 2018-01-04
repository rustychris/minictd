/* 
 *  Explore a wider variety of how to run the cell, basically
 *  trying some spectroscopy
 */
#include <SPI.h>
#include <SdFat.h>

#include "i2c_t3_local.h"
#include <TimeLib.h>

#include <ADC.h>
#include <DMAChannel.h>

#include "SeaDuck.h"
#include "ms5803.h"
#include "thermistor.h"
#include "conductivity.h"
#include "rtclock.h"
#include "SdFunctions.h"

SeaDuck logger;

void setup() {
  logger.setup();
}

void loop() {
  logger.loop();
}
