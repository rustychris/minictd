// defines related to the board layout
#define MAX_NUM_SENSORS 5

#define CDRIVE1 11
#define CDRIVE2 12
#define CCSENSE1 A12
#define CCSENSE2 A13
#define CVSENSE1 A10
#define CVSENSE2 A11

#define RSENSE 10
#define CELL_CONSTANT 15.0

// This gain reflects the inamp gain
#define CGAIN (10)

// a digital output tied to the enable pin of the LDO
#define POWER_3V3_ENABLE_PIN 3


#define NTC_SENSE A0
#define NTC_ADC ADC_0
#define NTC_GAIN 10 // from inamp
#define NTC_R_REF 120000.0 

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
