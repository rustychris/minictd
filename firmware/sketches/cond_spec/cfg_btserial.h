#define BT2S Serial1

// sets the default baud rate
// default rate is 9600, but definitely worth it to
// configurate at 115200.
#define BT2S_BAUD 115200

#ifdef HAS_GPS
# error Only one of GPS and Bluetooh Serial can be enabled.
#endif

