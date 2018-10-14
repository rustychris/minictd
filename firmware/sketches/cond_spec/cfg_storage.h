#ifdef CORE_TEENSY
// this is pin 12 of the schematic symbol, but in teensy land
// it's called pin 10
// 11,12 are out/in, 13 is SCK
#define SD_PIN_CS 10
#else
// Adalogger M0
#define SD_PIN_CS 4
#endif

#define SPI_SPEED SD_SCK_MHZ(4)

// Storage related settings:
#define CMDFILE "STARTUP.CMD"
// this MUST be a full 8.3 filename - like DATAxxxx.BIN
#define DATAFILETEMPLATE "DATAxxxx.BIN"
