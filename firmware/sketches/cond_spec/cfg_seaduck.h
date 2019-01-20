// defines related to the board layout
#define MAX_NUM_SENSORS 8

// feather board does not have these
// #include "cfg_cond.h"
// #include "cfg_ntc.h"

// #include "cfg_imu.h"
#include "cfg_pressure.h"
#include "cfg_rtc_ds3231.h"
// #include "cfg_gps.h"
#include "cfg_motor.h"


// if defined, a digital output tied to the enable pin of the LDO
// #define POWER_3V3_ENABLE_PIN 3

#include "cfg_storage.h"

#define STATUS_LED 8

#define STREAM_START_LINE "$"
#define MONITOR_START_LINE "#"

