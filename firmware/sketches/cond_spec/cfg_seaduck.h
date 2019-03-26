// defines related to the board layout
#define MAX_NUM_SENSORS 8

// feather board does not have these
// #include "cfg_cond.h"
// #include "cfg_ntc.h"

#include "cfg_imu.h"

// UNCOMMENT for GPS drifter
#include "cfg_gps.h"

// UNCOMMENT for neutral drifter
// #include "cfg_pressure.h"
// #include "cfg_rtc_ds3231.h"
// #include "cfg_motor.h"
// #include "cfg_buoyancy.h"
// #include "cfg_btserial.h"

#include "cfg_zmodem.h"

// -- Common --

// if defined, a digital output tied to the enable pin of the LDO
// #define POWER_3V3_ENABLE_PIN 3

#include "cfg_storage.h"


// for itsy m0 express
#define STATUS_LED 13

// specific to itsy m0 express
// other boards omit these
#define DOTSTAR_CLK 40
#define DOTSTAR_DATA 41

#define STREAM_START_LINE "$"
#define MONITOR_START_LINE "#"

