#include <Arduino.h>
#include "async_samd_wire.h"

uint8_t AsyncTwoWire::sendTransmission(void) {
  // endTransmission(); // DEV
  stopBit=true;

  // --original body of Wire.cpp endTransmission--
  transmissionBegun = false ;

  // need flag values from sam.h?
  // from home/.arduino15/packages/arduino/tools/CMSIS-Atmel/1.2.0/CMSIS/Device/ATMEL/samd21/include/component/sercom.h
  // SB and ERROR are also available.
  enable_int();

  async_status=IDLE;
  
  // Start I2C transmission
  // this will wait for idle, and taking ownership.
  // it then sends the address, and spins waiting for the MB (master on
  // bus) interrupt bit to get set.
  async_startTransmission(txAddress, WIRE_WRITE_FLAG);

  Serial.println("About to spin");
  while( (async_status==SEND_ADDRESS_WRITE) ||
         (async_status==WRITING) ) ;
  Serial.println("Exit spin");


  if (async_status == ERROR_NACK )
    {
      Serial.println("Got an error after sending address for a write");
      // this spins, but just for synchronization, probably don't have
      // to make that async
      sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
      clear_int();

      return 2 ;  // Address error
    }

  if (async_status != END_TRANSMISSION )
    {
      Serial.println("Got an error sending data");
      // this spins, but just for synchronization, probably don't have
      // to make that async
      sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
      clear_int();

      return 3 ;  // Nack or error
    }

  // // so far so good.
  // Serial.println("Successfully sent address for a write");
  // 
  // // Send all buffer
  // while( txBuffer.available() )
  //   {
  //     // Trying to send data
  //     // this will queue a byte, then spin waiting for MB to signal
  //     // that it has been sent.
  //     if ( !sercom->sendDataMasterWIRE( txBuffer.read_char() ) )
  //       {
  //         sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
  //         clear_int();
  // 
  //         return 3 ;  // Nack or error
  //       }
  //   }
  // 
  // if (stopBit)
  //   {
  //     sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
  //   }   

  clear_int(); // DEV

  Serial.println("End of sendTransmission");
  return 0;
}

void AsyncTwoWire::onService(void)
{
  uint8_t src;
  Serial.print("+");
  // signal that we got it, and are handling it.  Clear flag by
  // writing a 1 to it.
  if ( sercom->sercom->I2CM.INTFLAG.bit.MB ) {
    sercom->sercom->I2CM.INTFLAG.bit.MB=1;
    src=SERCOM_I2CM_INTFLAG_MB;
  } else if ( sercom->sercom->I2CM.INTFLAG.bit.SB ) {
    sercom->sercom->I2CM.INTFLAG.bit.SB=1;
    src=SERCOM_I2CM_INTFLAG_SB;
  } else if ( sercom->sercom->I2CM.INTFLAG.bit.ERROR ) {
    sercom->sercom->I2CM.INTFLAG.bit.ERROR=1;
    src=SERCOM_I2CM_INTFLAG_ERROR;
    Serial.println("ERROR!");
  }
  while ( sercom->sercom->I2CM.SYNCBUSY.bit.SYSOP != 0 ) ;

  // IDLE=>SEND_ADDRESS_WRITE
  // SEND_ADDRESS_WRITE=>ERROR_NACK
  // SEND_ADDRESS_WRITE=>WRITING
  // WRITING=> END_TRANSMISSION

  while(1) {
    switch ( async_status ) {
    case SEND_ADDRESS_WRITE:
      Serial.println("int: send_address");
      //ACK received (0: ACK, 1: NACK)
      if(sercom->sercom->I2CM.STATUS.bit.RXNACK){
        async_status=ERROR_NACK;
        return;
      } else {
        async_status=WRITING;
      }
      break; //superfluous with the loop, but for clear thinking
    case WRITING:
      // may need to enable error interrupt to get BUSERR here.
      if( sercom->sercom->I2CM.STATUS.bit.RXNACK ) {
        async_status=ERROR_NACK;
        return;
      }
      if( sercom->sercom->I2CM.STATUS.bit.BUSERR ) {
        async_status=ERROR_BUS;
        return;
      }

      if( txBuffer.available() ) {
        // sercom->sendDataMasterWIRE( txBuffer.read_char() );
        sercom->sercom->I2CM.DATA.bit.DATA = txBuffer.read_char();
        // there is more checking for BUSERR or RXNACK in the real code.
        return;
      } else {
        if (stopBit) sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
        async_status=END_TRANSMISSION;
      }
      return;
    case SEND_ADDRESS_READ:
      if ( src&0x02 ) { // SB
        async_status=SENT_ADDRESS_READ; // this gets changed 
        return;
      } else if (src&0x01) {
        // If the slave NACKS the address, the MB bit will be set.
        // In that case, send a stop condition and return false.
        sercom->sercom->I2CM.CTRLB.bit.CMD = 3; // Stop condition
        async_status=ERROR_NACK;
        return;
      }
    }
  }
}


// ----- Functions migrated from SERCOM.cpp -----

// Assumes that MB,SB interrupts are enabled.
void AsyncTwoWire::async_startTransmission(uint8_t address, SercomWireReadWriteFlag flag)
{
  Serial.println("async_startTransmission");

  if (flag==WIRE_WRITE_FLAG) 
    async_status=SEND_ADDRESS_WRITE;
  else
    async_status=SEND_ADDRESS_READ;

  // 7-bits address + 1-bits R/W
  address = (address << 0x1ul) | flag;

  // Wait idle or owner bus mode
  while ( !sercom->isBusIdleWIRE() && !sercom->isBusOwnerWIRE() );

  // Send start and address -- completion of this process triggers the
  // next interrupt
  sercom->sercom->I2CM.ADDR.bit.ADDR = address;
  if ( flag==WIRE_WRITE_FLAG ) 
    Serial.println("started send address for write");
  else
    Serial.println("started send address for read");
  

  // // Address Transmitted
  // if ( flag == WIRE_WRITE_FLAG ) // Write mode
  // {
  //   while( !sercom->sercom->I2CM.INTFLAG.bit.MB )
  //   {
  //     // Wait transmission complete
  //   }
  // }
  // else  // Read mode
  //   {
  //     while( !sercom->sercom->I2CM.INTFLAG.bit.SB )
  //       {
  //         // If the slave NACKS the address, the MB bit will be set.
  //         // In that case, send a stop condition and return false.
  //         if (sercom->sercom->I2CM.INTFLAG.bit.MB) {
  //           sercom->sercom->I2CM.CTRLB.bit.CMD = 3; // Stop condition
  //           return false;
  //         }
  //         // Wait transmission complete
  //       }
  //     
  //     // Clean the 'Slave on Bus' flag, for further usage.
  //     //sercom->I2CM.INTFLAG.bit.SB = 0x1ul;
  //   }
  // 
  // 
  // //ACK received (0: ACK, 1: NACK)
  // if(sercom->sercom->I2CM.STATUS.bit.RXNACK)
  //   {
  //     return false;
  //   }
  // else
  //   {
  //     return true;
  //   }
}
