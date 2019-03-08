#ifndef SERIALMUX_H
#define SERIALMUX_H

#include "FatLib/ArduinoStream.h"
#include "cfg_seaduck.h"

// A simple multiplexing stream for hardware/bluetooth serial and USB
// serial.  output is sent to both, number of bytes written
// is taken from bluetooth.  input comes from USB if it is active,
// otherwise from bluetooth.
class SerialMux : public Stream
{
public:
  uint32_t last_activity_millis;

  virtual void begin(int baud=115200) {
    last_activity_millis=millis();
    Serial.begin(baud);
#ifdef BT2S
    BT2S.begin(BT2S_BAUD);
#endif
  }
  Stream *src(void);  
  virtual int available(void) { return src()->available(); }
  virtual int peek(void);
  virtual int read(void);
  virtual void flush(void) { Serial.flush(); 
#ifdef BT2S
    // this causes issues regardless of how much data is being
    // written
    // BT2S.flush(); 
#endif
  }
  // This doesn't compile for Itsy M0 -- probably a different class for the
  // hardware serial.  
  //   virtual void clear(void) {
  // #ifdef BT2S
  //     BT2S.clear(); 
  // #endif
  //   }
#ifdef BT2S
  virtual size_t write(uint8_t c) { Serial.write(c); return BT2S.write(c); }
  virtual size_t write(const uint8_t *buffer, size_t size)
  { Serial.write(buffer, size); BT2S.write(buffer,size); return size; }
#else
  virtual size_t write(uint8_t c) { return Serial.write(c); }
  virtual size_t write(const uint8_t *buffer, size_t size)
  { Serial.write(buffer, size); return size; }
#endif

  size_t write(unsigned long n)   { return write((uint8_t)n); }
  size_t write(long n)            { return write((uint8_t)n); }
  size_t write(unsigned int n)    { return write((uint8_t)n); }
  size_t write(int n)             { return write((uint8_t)n); }
  size_t write(const char *str)	{ size_t len = strlen(str);
    write((const uint8_t *)str, len);
    return len; }

  // no way to know if BT is connected right now -- so defer
  // Serial.
  operator bool() { return (bool)Serial; }
};

extern SerialMux mySerial;

extern ArduinoOutStream cout;

#endif // SERIALMUX_H
