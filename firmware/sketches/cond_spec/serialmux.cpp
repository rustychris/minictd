#include "serialmux.h"

SerialMux mySerial;

// Originally just for SdFunctions, but maybe it would be useful more
// generally
ArduinoOutStream cout(mySerial);

int SerialMux::read(void) { 
  int res=src()->read();
  if (res>=0)
    last_activity_millis=millis();
  return res;
}

int SerialMux::peek(void) {
  return src()->peek();
}

Stream *SerialMux::src(void) {
#ifdef BT2S
  // This worked on the teensy, but dtr() not defined for itsy.
  // Stream *src(void) { return  ( Serial.dtr() ? ((Stream*)&Serial) : ((Stream*)&BT2S)); }
  // Trying this as a more general approach:
  if ( (bool)Serial && Serial.available() ) {
    return (Stream*)&Serial;
  } else {
    return  (Stream*)&BT2S;
  }
#else
  return (Stream*)&Serial;
#endif
}
