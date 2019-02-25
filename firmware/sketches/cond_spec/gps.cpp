// Serial gps interface
#include "cfg_seaduck.h"
#ifdef HAS_GPS
#include <Arduino.h>

#include "gps.h"

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
  // Serial.println("GPS async_read not implemented");
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
      //Serial.print("NMEA (read=");
      //Serial.print(read_sentence);
      //Serial.print(" write=");
      //Serial.print(write_sentence);
      //Serial.print("): ");
      Serial.print("NMEA=");
      Serial.print(sentence_buff[read_sentence]); // has its own newline, I think
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
  Serial.println("# Exiting gps watch loop. ");
}

void GPS::record_byte(char c) {
  if((c=='$') || (write_char>=MAX_SENTENCE_LENGTH-1)) {
    // null terminate that 
    sentence_buff[write_sentence][write_char]=0;
    write_sentence=(write_sentence+1)%MAX_NMEA_SENTENCES;
    write_char=0;
    if ( read_sentence==write_sentence ) {
      // Serial.println("Write overrun");
      // the write pointer has overtaken the read pointer.
      // favor new data over old, so we bump the read pointer
      // TODO: Check for race conditions
      read_sentence=(read_sentence+1)%MAX_NMEA_SENTENCES;
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
      Serial.print("gps_enable="); Serial.println( enabled );
    }
  } else {
    return false;
  }
  return true;
}

void GPS::help(void) {
  Serial.println("  GPS");
  Serial.println("    gps_watch        # stream GPS serial data");
  Serial.println("    gps_enable[=0,1] # enable/disable logging of gps");
}

#define XSTR(s) STR(s)
#define STR(s) #s

void GPS::write_frame_info(Print &out) {
  out.print("('gps_nmea','U" XSTR(MAX_SENTENCE_LENGTH) "'," XSTR(MAX_NMEA_SENTENCES) "),");
}

void GPS::write_data(Print &out) {
  int count;
  static char empty[MAX_SENTENCE_LENGTH]="";
  while(read_sentence!=write_sentence) {
    write_base16(out,(uint8_t*)&sentence_buff[read_sentence],MAX_SENTENCE_LENGTH);
    // TODO: check for race conditions with writer.
    read_sentence=(read_sentence+1)%MAX_NMEA_SENTENCES;
    count++;
  }
  // pad out others
  for(;count<MAX_NMEA_SENTENCES;count++ ) {
    write_base16(out,(uint8_t*)empty,MAX_SENTENCE_LENGTH);
  }
}


#endif // HAS_GPS

