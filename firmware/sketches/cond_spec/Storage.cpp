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
// What is the difference between BUFFER_BLOCK_COUNT
// and QUEUE_DIM?  This code has an extra level of
// indirection that we probably don't need.
const uint8_t BUFFER_BLOCK_COUNT = 30; 
const uint16_t DATA_DIM = 512;  

struct block_t {
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
    for(int i=0;i<20;i++) {
      sd.initErrorPrint();
      delay(500);
    }
    status=NOCARD;
  } else {
    status=DISABLED;
  }

  // And initialize the block queues
  emptyHead = emptyTail = 0;
  fullHead = fullTail = 0;
  
  // initialize ISR
  isrBuf = 0; 
  isrOver = 0;
    
  // put rest of buffers in empty queue
  // relies on fullQueue and emptyQueue being initialized to 0
  // by the compiler.
  for (uint8_t i = 0; i < BUFFER_BLOCK_COUNT; i++) {
    emptyQueue[emptyHead] = (block_t*)(block + 512 * i);
    emptyHead = queueNext(emptyHead);
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
  status=ENABLED;
  sync_counter=0; // counter for periodically flushing data
}

void Storage::close_file(void) {
  status=DISABLED;
  myFile.close();
}

/* 
  Find the first filename of the form dataNNNN.bin which doesn't
  exist, starting at data0000.bin
 */
void Storage::set_next_active_filename(void) {
  if(status==NOCARD) {
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
            Serial.print("# Logging to ");
            Serial.println(active_filename);
            return;
          }
        }
      }
    }
  }
}

// this is called periodically to flush full buffers to SD
void Storage::loop(void) {
  // Loop a finite number of times - 
  // if we're swamped, better to occasionally come up for air
  // and see if there is input waiting to stop the process
  for( uint16_t write_loops=0; 
       (fullHead != fullTail) && write_loops < BUFFER_BLOCK_COUNT;
       write_loops++ ) {
    // block to write
    block_t* block = fullQueue[fullTail];
    sync_counter++; // one step closer to needing to sync()

    if ( status==ENABLED ) {
      if (!myFile.write((void*)block,sizeof(block_t))) {
        sd.errorHalt("failed to write");
      }
    }
    
    // move block to empty queue
    emptyQueue[emptyHead] = block;
    emptyHead = queueNext(emptyHead);
    fullTail = queueNext(fullTail);

    if ( status==ENABLED ) {
      if ( sync_counter > sync_interval_blocks ) {
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
    close_file();
  }
}

// /** call this when a sample has been converted - will queue it in the
//     right buffer.  safe for calling inside ISR */
// void Storage::store_frame(uint8_t *frame) {
//   // experiment to get low-latency serial output
//   // if there is no backlog of blocks to be written, then
//   // go ahead and flush this block so it can be written
//   // sooner
//   // if ( isrBuf && log_to_serial && (fullHead == fullTail) ) {
//   //   close_block();
//   // }
//   
//   // Get an appropriate block
// 
//   // // switch to a data block if necessary
//   // if( isrBuf && ( (isrBuf->flags&FLAG_TYPE_MASK) == FLAG_TYPE_TEXT) ) {
//   //   close_block();
//   // }
//   
//   if ( !isrBuf ) {
//     open_block();
//     if(!isrBuf) return;
//   }
// 
//   memcpy(&(isrBuf->data[isrBuf_pos]),
//          frame,
//          frame_bytes);
//   isrBuf_pos+=frame_bytes;
//   // isrBuf->frame_count++;
// 
//   // if no room for another frame, mark this one full
//   if (DATA_DIM < isrBuf_pos+frame_bytes ) {
//     close_block();
//   }
// }

// only call when isrBuf==0 !
void Storage::open_block() {
  if (emptyHead != emptyTail) {
    // remove buffer from empty queue
    isrBuf = emptyQueue[emptyTail];
    emptyTail = queueNext(emptyTail);
    isrBuf_pos=0;
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
  if ( !isrBuf ) 
    open_block();
  if ( !isrBuf ) return 0;

  isrBuf->data[isrBuf_pos++] = b;
  if( isrBuf_pos>=DATA_DIM ) // really should never be greater
    close_block();
  return 1;
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
  // Serial.print("log_to_serial: ");
  // Serial.println(log_to_serial);
}

bool confirm(void) {
  // be extra careful that we don't accidentally format
  // discard any extraneous characters:
  while(Serial.available()) Serial.read();
  
  Serial.println("Are you sure you want to proceed? Type y to confirm");

  while(!Serial.available()) ; // wait for a key press
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
  } else if ( strcmp(cmd,"sd_open")==0) {
    open_next_file();
    Serial.print("# opened ");
    Serial.println(active_filename);
  } else if ( strcmp(cmd,"sd_close")==0) {
    close_file();
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
  Serial.println("    sd_open     # open next file for output");
  Serial.println("    sd_close    # close current output file");
}
