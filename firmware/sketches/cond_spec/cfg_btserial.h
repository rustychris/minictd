#define BT2S Serial1

// sets the default baud rate
#define BT2S_BAUD 9600

#ifdef HAS_GPS
# error Only one of GPS and Bluetooh Serial can be enabled.
#endif

