#ifndef I2C_M0_LOCAL
#define I2C_M0_LOCAL
// #include <Wire.h> // Is this a problem with redefining Wire below?
// using a homemade Wire -
// #include "wiring_private.h" // pinPeripheral() function

#include "I2C_DMAC.h"
#define ASYNC_I2C

class MyI2C {
public:
  MyI2C(void) {
    ;
  };
  void begin(void) {
    I2C.begin(400000);      // Start I2C bus at 400kHz          
    I2C.setWriteChannel(2); // Set the I2C1 DMAC write channel to 2
    I2C.setReadChannel(3);  // Set the I2C1 DMAC read channel to 3
  }

  uint8_t _address;
  uint8_t read_ptr,write_ptr;
  uint8_t buff_len;
  uint8_t buffer[100];
  void sendRequest(uint8_t address,int count) {
    read_ptr=0;
    buff_len=count; // premature...
		I2C.readBytes(address, buffer, count);
  }
  void requestFrom(uint8_t address,int count) {
    sendRequest(address,count);
    while(I2C.readBusy) ;
  };
  uint8_t read(void) {
    if(read_ptr<buff_len) {
      return buffer[read_ptr++];
    }
    Serial.println("Read past end of buffer");
    return 0;
  }
  bool available(void) {
    return read_ptr<buff_len;
  }

  uint8_t xmit_address;
  void beginTransmission(uint8_t address) {
    _address=address;
    write_ptr=0;
  }
  uint8_t write(uint8_t data) {
    buffer[write_ptr]=data;
    write_ptr++;
    return 1;
  }
  void onTransmitDone(void (*callback)(void)) {
    I2C.attachWriteCallback((voidFuncPtr)callback);
  }
  void onReqFromDone(void (*callback)(void)) {
    I2C.attachReadCallback((voidFuncPtr)callback);
  }
  void sendTransmission(void) {
    I2C.initWriteBytes(_address, buffer, write_ptr);
    I2C.write();
  }
  uint8_t endTransmission(void) {
    sendTransmission();
    // Serial.println("endTransmission: write spinning");
    while(I2C.writeBusy);
    // Serial.println("endTransmission: exit spinning");
    return 0; // not sure how to get proper status
  }
};
extern MyI2C AWire;

#endif // I2C_M0_LOCAL
