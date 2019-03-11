// Serial gps interface
#include "cfg_seaduck.h"
#ifdef HAS_GPS
#include <Arduino.h>

#include "gps.h"
#include "serialmux.h"

#ifdef BT2S
# error Bluetooth serial cannot coexist with GPS
#endif
// Getting serial interrupts requires adding this to variant.cpp in place
// of the existing SERCOM0_Handler()
//    void Serial1_callback() __attribute__((weak)); // RH
//    void Serial1_callback() {}
//    
//    void SERCOM0_Handler()
//    {
//      Serial1.IrqHandler();
//      Serial1_callback();
//    }

// Then here we can do this mangled business:
GPS *serial1_gps=NULL;
void Serial1_callback() {
  if (serial1_gps!=NULL) {
    serial1_gps->serial_callback();
  }
}

void GPS::init(void){
  Serial1.begin(9600);
  Serial.println("initialized Serial1");
  serial1_gps=this;
}

void GPS::async_read(void) {
  // this is always going in the background.
  pop_fn_and_call();
}

void GPS::serial_callback() {
  while ( Serial1.available() ) {
    record_byte(Serial1.read());
  }
}

void GPS::watch(void) {
  char c;
  while(1) {
    // If interrupt handler is not in place (or if we win the race condition)
    // if ( Serial1.available() ) {
    //   c=Serial1.read();
    //   Serial.write(c);
    // }

    // With interrupt handler, should see them here
    while(read_sentence!=write_sentence) {
      mySerial.print("NMEA=");
      mySerial.print(sentence_buff[read_sentence]); // has its own newline, I think
      read_sentence=(read_sentence+1)%MAX_NMEA_SENTENCES;
    }

    // Allow stopping the loop on ! or ESC
    if( Serial.available() ) {
      c=Serial.read();
      // stop it when an exclamation or ESC is read
      if ( (c=='!') || (c==27) ) {
        break;
      } 
    }
  }
  mySerial.println("# Exiting gps watch loop. ");
}

void GPS::record_byte(char c) {
  if((c=='$') || (write_char>=MAX_SENTENCE_LENGTH-1)) {
    // null terminate that 
    sentence_buff[write_sentence][write_char]=0;
    // this is when we decide whether to keep this sentence
    
    if( false 
#ifdef IGNORE_NMEA_GPGSV
        || strncmp(sentence_buff[write_sentence],"$GPGSV",6)==0
#endif
#ifdef IGNORE_NMEA_GPGGA
        || strncmp(sentence_buff[write_sentence],"$GPGGA",6)==0
#endif
#ifdef IGNORE_NMEA_GPGLL
        || strncmp(sentence_buff[write_sentence],"$GPGLL",6)==0
#endif
#ifdef IGNORE_NMEA_GPVTG
        || strncmp(sentence_buff[write_sentence],"$GPVTG",6)==0
#endif
        ) {
      // Will ignore this sentence -- keep write_sentece the same
      // and start overwriting at 0.
      write_char=0;
    } else {
      // move on to next sentence
      write_sentence=(write_sentence+1)%MAX_NMEA_SENTENCES;
      write_char=0;
      if ( read_sentence==write_sentence ) {
        // mySerial.println("Write overrun");
        // the write pointer has overtaken the read pointer.
        // favor new data over old, so we bump the read pointer
        // TODO: Check for race conditions
        read_sentence=(read_sentence+1)%MAX_NMEA_SENTENCES;
      }
    }
  }

  sentence_buff[write_sentence][write_char]=c;
  write_char++;
}

bool GPS::dispatch_command(const char *cmd, const char *cmd_arg) {
  if( !strcmp(cmd,"gps_watch") ) {
    watch();
  } else if(!strcmp(cmd,"gps_enable")) {
    if(cmd_arg) {
      enabled=(bool)atoi(cmd_arg);
    } else {
      mySerial.print("gps_enable="); mySerial.println( enabled );
    }
  } else {
    return false;
  }
  return true;
}

void GPS::help(void) {
  mySerial.println("  GPS");
  mySerial.println("    gps_watch        # stream GPS serial data");
  mySerial.println("    gps_enable[=0,1] # enable/disable logging of gps");
}

#define XSTR(s) STR(s)
#define STR(s) #s

void GPS::write_frame_info(Print &out) {
  // used to use U, but binary interpretation in numpy requires
  // S, which implies byte string
  out.print("('gps_nmea','S" XSTR(MAX_SENTENCE_LENGTH) "'," XSTR(MAX_NMEA_SENTENCES) "),");
}

void GPS::write_data(Print &out) {
  static char empty[MAX_SENTENCE_LENGTH]="";
  int write_sentence_copy;

  int count=0;

  // make a copy to avoid race conditions
  // there is still the possibility of race conditions if the ring
  // buffer overruns.
  noInterrupts();
  write_sentence_copy=write_sentence;
  interrupts();
  
  while(read_sentence!=write_sentence_copy) {
    // This used to have &sentence_buff[read_sentence], but that seems
    // unnecessary?
    write_base16(out,(uint8_t*)sentence_buff[read_sentence],MAX_SENTENCE_LENGTH);
    memset((void*)(&sentence_buff[read_sentence]),0,MAX_SENTENCE_LENGTH);
    
    // TODO: double check for race conditions with writer.
    noInterrupts();    
    read_sentence=(read_sentence+1)%MAX_NMEA_SENTENCES;
    interrupts();
    
    count++;
  }
  // pad out others
  for(;count<MAX_NMEA_SENTENCES;count++ ) {
    write_base16(out,(uint8_t*)empty,MAX_SENTENCE_LENGTH);
  }
}


#endif // HAS_GPS

