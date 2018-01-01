/**** SD logging code, stolen from fat16lib's AnalogIsrLogger ****/
/* The logging portion of Storage */

/*  mainly to get efficient block writes */
#include <Arduino.h>

#include "seaduck_cfg.h"
#include <SPI.h>
#include <SdFat.h>

#include "SdFunctions.h"
#include "SeaDuck.h"

SdFat sd;     // for logging
SdFile myFile;// for logging

// tradeoff of latency tolerance versus RAM usage.
// at 10kHz logging, 12 got some overruns.
const uint8_t BUFFER_BLOCK_COUNT = 30; 
const uint16_t DATA_DIM = 504;  

#define FLAG_TYPE_MASK 1
#define FLAG_TYPE_DATA 0
#define FLAG_TYPE_TEXT 1
#define FLAG_OVERRUN 2

struct block_t {
  uint32_t unixtime; 
  uint16_t ticks;
  uint8_t frame_count;
  uint8_t flags;

  uint8_t data[DATA_DIM];
};

// queues of 512 byte SD blocks
const uint8_t QUEUE_DIM = 32;  // Must be a power of two!

block_t* isrBuf;
uint16_t isrBuf_pos; // byte offset into data attribute

uint16_t isrOver = 0;

block_t* emptyQueue[QUEUE_DIM];
uint8_t emptyHead;
uint8_t emptyTail;

block_t* fullQueue[QUEUE_DIM];
uint8_t fullHead;
uint8_t fullTail;


// allocate buffer space
// note that this is always used, regardless of whether we're sampling or 
// not.  If more functions are added which want some RAM when not 
// sampling, this will have to be changed
uint8_t block[512 * BUFFER_BLOCK_COUNT];

inline uint8_t queueNext(uint8_t ht) {return (ht + 1) & (QUEUE_DIM -1);}

void Storage::init(void) {
  // this is called once on boot up
  // And initialize the SD interface and open a file
  
  if (!sd.begin(SD_PIN_CS, SPI_SPEED)) {
    // rather than halt, go into a loop repeating the message 
    // to make it easier to catch on a serial console
    // beep_code(Logger::NO_SD_CARD);
    for(int i=0;i<20;i++) {
      sd.initErrorPrint();
      delay(500);
    }
    status=NOCARD;
  } else {
    status=ENABLED;
  }
}

// This is called before sampling begins, setting up the buffers
// and in the future possibly pre-allocating files.
void Storage::setup(void) {
  // initialize queues
  emptyHead = emptyTail = 0;
  fullHead = fullTail = 0;
  
  // initialize ISR
  isrBuf = 0;
  isrOver = 0;

  // possible to use the SdFat buffer for a block, 
  // but try keeping the logic here independent of that level
  // of internals, and just pay the price of some copying
  
  // put rest of buffers in empty queue
  // relies on fullQueue and emptyQueue being initialized to 0
  // by the compiler.
  for (uint8_t i = 0; i < BUFFER_BLOCK_COUNT; i++) {
    emptyQueue[emptyHead] = (block_t*)(block + 512 * i);
    emptyHead = queueNext(emptyHead);
  }

  if( ! log_to_serial ) {
    if (status==ENABLED)
      open_next_file();
  }

  overruns = 0;
  frame_count = 0;
}

void Storage::open_next_file(void) {
  set_next_active_filename();
  if (!myFile.open(active_filename, O_RDWR | O_CREAT)) {
    sd.errorHalt("opening output file for write failed");
  } 
  time_t unixtime=logger.unixtime();
  myFile.timestamp(T_ACCESS|T_CREATE|T_WRITE,
                   // dt.year(),dt.month(),dt.day(),dt.hour(),dt.minute(),dt.second());
                   year(unixtime), month(unixtime), day(unixtime),hour(unixtime),
                   minute(unixtime),second(unixtime));
  sync_counter=0;
}

/* 
  Find the first filename of the form dataNNNN.bin which doesn't
  exist, starting at data0000.bin
 */
void Storage::set_next_active_filename(void) {
  if(status==DISABLED) {
    strcpy(active_filename,"--DISABLED--");
    return;
  } else if(status==NOCARD) {
    strcpy(active_filename,"---NOCARD---");
    return;
  }

  strcpy(active_filename,DATAFILETEMPLATE);

  //kludgy way of formatting strings
  for(active_filename[4]='0';active_filename[4]<='9';active_filename[4]++) {
    for(active_filename[5]='0';active_filename[5]<='9';active_filename[5]++) {
      for(active_filename[6]='0';active_filename[6]<='9';active_filename[6]++) {
        for(active_filename[7]='0';active_filename[7]<='9';active_filename[7]++) {
          if( ! sd.exists(active_filename) ) {
            Serial.print("Logging to ");
            Serial.println(active_filename);
            return;
          }
        }
      }
    }
  }
}


//--- BELOW HERE things need to be switched to async

// this is called periodically to flush full buffers to SD
void Storage::loop(void) {
  // Loop a finite number of times - 
  // if we're swamped, better to occasionally come up for air
  // and see if there is input waiting to stop the process
  for( uint16_t write_loops=0; 
       (fullHead != fullTail) && write_loops <  BUFFER_BLOCK_COUNT;
       write_loops++ ) {
    // block to write
    block_t* block = fullQueue[fullTail];
    sync_counter++;

    if( log_to_serial ) {
      for(int fidx=0;fidx<block->frame_count;fidx++) {
        Serial.print(STREAM_START_LINE);
        for(int i=0;i<frame_bytes;i++){
          uint8_t byte=block->data[fidx*frame_bytes+i];
          // unfortunately, Serial drops leading 0, so manually pad each byte to 2 hex digits.
          if( byte<0x10 ) 
            Serial.print("0");
          Serial.print(byte,HEX);
        }
        Serial.println();
      }
    } else {
      if ( status==ENABLED ) {
        if (!myFile.write((void*)block,sizeof(block_t))) {
          sd.errorHalt("failed to write");
        }
      }
    }
    frame_count += block->frame_count;
    // check for overrun - doesn't count them, just flags
    // that there were some.
    if (block->flags & FLAG_OVERRUN) {
      overruns += 1;
    }
    // move block to empty queue
    emptyQueue[emptyHead] = block;
    emptyHead = queueNext(emptyHead);
    fullTail = queueNext(fullTail);

    if ( status==ENABLED ) {
      if ( !log_to_serial && (sync_counter > sync_interval_blocks) ) {
        myFile.sync();
        sync_counter=0;
      }
    }
  }
}

// call this when done with sampling, and the sample ISR
// is no longer running.
void Storage::cleanup(void) {
  // on exit from sample loop
  if (isrBuf != 0) {
    close_block();
  }
  loop(); // write any straggling data
  if ( status==ENABLED ) {
    if ( !log_to_serial )
      myFile.close();
  }
}


/** call this when a sample has been converted - will queue it in the
    right buffer.  safe for calling inside ISR */
void Storage::store_frame(uint8_t *frame) {
  // experiment to get low-latency serial output
  // if there is no backlog of blocks to be written, then
  // go ahead and flush this block so it can be written
  // sooner
  if ( isrBuf && log_to_serial && (fullHead == fullTail) ) {
    close_block();
  }
  // Get an appropriate block

  // switch to a data block if necessary
  if( isrBuf && ( (isrBuf->flags&FLAG_TYPE_MASK) == FLAG_TYPE_TEXT) ) {
    close_block();
  }
  if ( !isrBuf ) {
    open_block(FLAG_TYPE_DATA);
    if(!isrBuf) return;
  }

  memcpy(&(isrBuf->data[isrBuf_pos]),
         frame,
         frame_bytes);
  isrBuf_pos+=frame_bytes;
  isrBuf->frame_count++;

  // if no room for another frame, mark this one full
  if (DATA_DIM < isrBuf_pos+frame_bytes ) {
    close_block();
  }
}

// only call when isrBuf==0 !
void Storage::open_block(uint8_t flags) {
  if (emptyHead != emptyTail) {
    // remove buffer from empty queue
    isrBuf = emptyQueue[emptyTail];
    emptyTail = queueNext(emptyTail);
    // initialize block:
    isrBuf->frame_count = 0; // no frames
    isrBuf->unixtime=logger.unixtime(); // logger.unixtime;
    isrBuf_pos=0;
#ifdef RTC_ENABLE
    isrBuf->ticks=logger.rtc_pulse_count;
#endif
    isrBuf->flags=flags;
  } else {
    // no buffers - count overrun
    if (isrOver < 0XFF) isrOver++;
  }
}

// only call when isrBuf!=0
void Storage::close_block(void){
  // put buffer isrIn full queue
  if( isrBuf_pos < DATA_DIM ) {
    memset(isrBuf->data+isrBuf_pos,0,DATA_DIM-isrBuf_pos);
  }

  fullQueue[fullHead] = isrBuf;
  fullHead = queueNext(fullHead);
    
  //set buffer needed and clear overruns
  isrBuf = 0;
  isrBuf_pos=0;
  isrOver = 0;
}

size_t Storage::write(uint8_t b) {
  // switch to a text block if necessary
  if( isrBuf &&
      ( (isrBuf->flags&FLAG_TYPE_MASK) == FLAG_TYPE_DATA) ) {
    close_block();
  }

  if ( !isrBuf ) 
    open_block(FLAG_TYPE_TEXT);
  if ( !isrBuf ) return 0;

  isrBuf->data[isrBuf_pos++] = b;
  // last byte should be left 0 as null termination for the
  // string
  if( isrBuf_pos>=DATA_DIM-1 ) {
    isrBuf_pos=DATA_DIM-1; // just in case isrBuf_pos got crazy
    close_block();
  }
  return 1;
}

uint8_t Storage::send_data(const char *filename,uint32_t start,uint32_t bytes) {
  // send a portion of a file over the serial link
  // the data are hex-encoded, and
  // after the data, an additional 8-bit checksum is sent, also hex-encoded.
  

  // returns 0 on success
  uint8_t checksum=0;
  
  if (!myFile.open(filename, O_READ )) {
    Serial.print("Failed to open ");
    Serial.println(filename);
    return 1;
  }
  myFile.seekSet(start);

  if ( bytes==0 ) {
    bytes=myFile.fileSize();
  }

  uint8_t c;

  for(uint32_t i=0;i<bytes;i++) {
    myFile.read(&c,1);
    if( c<0xF ) Serial.write('0');
    Serial.print(c,HEX);
    if((i>0) && ((i & 0x3F) == 0x3F) ) 
      Serial.println("");
    checksum+=c;
  }
  Serial.println("");
  if( checksum<0xF ) Serial.write('0');
  Serial.println(checksum,HEX);
  
  myFile.close();
  return 0;
}

void Storage::info() 
{
  Serial.print("storage_status: ");
  Serial.println(status);
  Serial.print("storage_status_name: ");
  if (status==DISABLED) {
    Serial.println("DISABLED");
  } else if (status==NOCARD) {
    Serial.println("NOCARD");
  } else if (status==ENABLED) {
    Serial.println("ENABLED");
  }
  Serial.print("log_to_serial: ");
  Serial.println(log_to_serial);
}

bool confirm(void) {
  while(!Serial.available()) ;

  Serial.println("Are you sure you want to proceed? Type y to confirm");

  char key = Serial.read(); 
  
  return key == 'y';
}

bool Storage::dispatch_command(const char *cmd, const char *cmd_arg) {
  if(strcmp(cmd,"erase")==0) {
    if( confirm() )
      format('E');
    else {
      Serial.println("Aborted");
    }
  } else if ( strcmp(cmd,"format")==0 ) { 
    if( confirm() )
      format('F');
    else
      Serial.println("Aborted");
  } else if ( strcmp(cmd,"quickformat")==0 ) {
    if( confirm() )
      format('Q');
    else
      Serial.println("Aborted");
  } else if ( strcmp(cmd,"ls")==0) {
    sd.ls(&Serial,LS_SIZE);
  } else if ( strcmp(cmd,"sd_status")==0) {
    info();
  } else {
    return false;
  }
  return true;
}

void Storage::help() {
  Serial.println("  Storage");
  Serial.println("    erase       # erase SD card");
  Serial.println("    format      # format SD card");
  Serial.println("    quickformat # as advertised");
  Serial.println("    ls          # show files on sd card");
  Serial.println("    sd_status   # display SD info");
}
