// defines related to the board layout
#define MAX_NUM_SENSORS 5

// feather board does not have these
// #include "cfg_cond.h"
// #include "cfg_ntc.h"

#include "cfg_rtc.h" // clock source configured within

// if defined, a digital output tied to the enable pin of the LDO
// #define POWER_3V3_ENABLE_PIN 3

// this is pin 12 of the schematic symbol, but in teensy land
// it's called pin 10
// 11,12 are out/in, 13 is SCK
#define SD_PIN_CS 10

// Storage related settings:
#define CMDFILE "STARTUP.CMD"
// this MUST be a full 8.3 filename - like DATAxxxx.BIN
#define DATAFILETEMPLATE "DATAxxxx.BIN"
#define STREAM_START_LINE "$"
#define MONITOR_START_LINE "#"

#define SPI_SPEED SD_SCK_MHZ(4)
