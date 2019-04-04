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

void SerialMux::pass_through(void) {
  // temporarily create a pipe between usb serial and bluetooth serial,
  // so that usb input is written to bluetooth serial, and bluetooth
  // input is written to usb.
#ifdef BT2S
  // This is generally used for AT mode, so don't send this message
  // to the bluetooth side
  BT2S.begin(38400); // AT mode always 38400
  
  Serial.println("Entering USB<->Bluetooth.  Exit by typing '~' from either side");
  int data;
  bool active=true;
  
  while (active) {
    while ( Serial.available() ) {
      data=Serial.read();
      if ((char)data == '~')
        active=false;
      else if( data>=0 ) {
        BT2S.write((char)data);
      }
    }
    while ( BT2S.available() ) {
      data=BT2S.read();
      if ((char)data == '~')
        active=false;
      else if( data>=0 ) {
        Serial.write((char)data);
      }
    }
  }
  Serial.println("Exit USB<->Bluetooth.");
  BT2S.begin(bt_baud); // AT mode always 38400

#else // BT2S
  Serial.println("No BT serial.");
#endif
}
