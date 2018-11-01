#ifndef ASYNC_SAMD_WIRE
#define ASYNC_SAMD_WIRE

#include <Wire.h>
typedef enum {
  IDLE,
  SEND_ADDRESS_WRITE,
  SEND_ADDRESS_READ,
  WRITING,
  SENT_ADDRESS_READ, // to be replaced by READING
  ERROR_NACK,
  ERROR_BUS,
  END_TRANSMISSION // is this different from IDLE?
} AsyncStatus;

class AsyncTwoWire : public TwoWire {
 private:
  volatile AsyncStatus async_status;
  bool stopBit; // default in Wire.cpp

 public:
 AsyncTwoWire(SERCOM *sc,int pin_sda, int pin_scl) : TwoWire(sc,pin_sda,pin_scl) {}
  uint8_t sendTransmission(void);
  // only while developing sendTransmission
  uint8_t endTransmission(void) { return sendTransmission(); }
  void onService(void);

  // duplicated, non-blocking versions of functions from SERCOM.cpp
  void async_startTransmission(uint8_t address, SercomWireReadWriteFlag flag);

  void enable_int(void) {
    Serial.println("Enable ints");
    sercom->sercom->I2CM.INTENSET.reg=SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB |
      SERCOM_I2CM_INTENSET_ERROR;
    // Wait the SYSOP bit from SYNCBUSY to come back to 0
    while ( sercom->sercom->I2CM.SYNCBUSY.bit.SYSOP != 0 ) ;
  }
  void clear_int(void) {
    Serial.println("Disable ints");
    sercom->sercom->I2CM.INTENCLR.reg=SERCOM_I2CM_INTENCLR_MB | SERCOM_I2CM_INTENCLR_SB |
      SERCOM_I2CM_INTENCLR_ERROR;
    // Wait the SYSOP bit from SYNCBUSY to come back to 0
    while ( sercom->sercom->I2CM.SYNCBUSY.bit.SYSOP != 0 ) ;
  }

};

#endif // ASYNC_SAMD_WIRE
