/* 
 *  Explore a wider variety of how to run the cell, basically
 *  trying some spectroscopy
 */
#include <SPI.h>
#include <SD.h>

// #include <Audio.h>
#include <ADC.h>
#include <DMAChannel.h> // used to be in quotes
// #include <utility/pdb.h>

#include "SeaDuck.h"

SeaDuck seaduck;

void setup() {
  seaduck.setup();
}

void loop() {
  seaduck.loop();
}
