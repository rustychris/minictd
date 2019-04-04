/**** SD logging code, stolen from fat16lib's AnalogIsrLogger ****/
/* The logging portion of Storage */

/*  mainly to get efficient block writes */
#include <Arduino.h>

#include "cfg_seaduck.h"
#include "serialmux.h"

#include <SPI.h>
#include <SdFat.h>

#include "SdFunctions.h"
#include "SeaDuck.h"

#ifdef HAS_ZMODEM
#include "zmodem.h"
extern int Filesleft;
extern long Totalleft;
SdFile fout; // specific to zmodem interface
#endif

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
            mySerial.print("# Logging to ");
            mySerial.println(active_filename);
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
  if ( !isrBuf ) {
    overruns++;
    return 0;
  }

  isrBuf->data[isrBuf_pos++] = b;
  if( isrBuf_pos>=DATA_DIM ) // really should never be greater
    close_block();
  return 1;
}

void Storage::info() 
{
  mySerial.print("storage_status=");
  mySerial.println(status);
  mySerial.print("storage_status_name=");
  if (status==DISABLED) {
    mySerial.println("DISABLED");
  } else if (status==NOCARD) {
    mySerial.println("NOCARD");
  } else if (status==ENABLED) {
    mySerial.println("ENABLED");
  }
  mySerial.print("overruns=");
  mySerial.println(overruns);
}

bool confirm(void) {
  // be extra careful that we don't accidentally format
  // discard any extraneous characters:
  while(mySerial.available()) mySerial.read();
  
  mySerial.println("Are you sure you want to proceed? Type y to confirm");

  while(!mySerial.available()) ; // wait for a key press
  char key = mySerial.read(); 

  return key == 'y';
}

bool Storage::dispatch_command(const char *cmd, const char *cmd_arg) {
  if(strcmp(cmd,"erase")==0) {
    if( confirm() )
      format('E');
    else {
      mySerial.println("Aborted");
    }
  } else if ( strcmp(cmd,"format")==0 ) { 
    if( confirm() )
      format('F');
    else
      mySerial.println("Aborted");
  } else if ( strcmp(cmd,"quickformat")==0 ) {
    if( confirm() )
      format('Q');
    else
      mySerial.println("Aborted");
  } else if ( strcmp(cmd,"ls")==0) {
    sd.ls(&mySerial,LS_SIZE);
  } else if ( strcmp(cmd,"sd_status")==0) {
    info();
  } else if ( strcmp(cmd,"sd_open")==0) {
    open_next_file();
    mySerial.print("# opened ");
    mySerial.println(active_filename);
  } else if ( strcmp(cmd,"sd_close")==0) {
    close_file();
#ifdef HAS_ZMODEM
  } else if ( strcmp(cmd,"sz")==0) {
    zmodem_send_file(cmd_arg);
#endif // HAS_ZMODEM
#ifdef HAS_XMODEM
  } else if ( strcmp(cmd,"sx")==0) {
    xmodem_send_file(cmd_arg,XMODEM);
  } else if ( strcmp(cmd,"sb")==0) {
    xmodem_send_file(cmd_arg,YMODEM);
#endif // HAS_XMODEM
  } else {
    return false;
  }
  return true;
}

#ifdef HAS_ZMODEM
void Storage::zmodem_send_file(const char *filename) {
  if (!fout.open(filename, O_READ)) {
    mySerial.println("file.open failed");
    return;
  }

  // Zmodem notes:
  // from zmodem.ino:

  // To send a file from the arduino to host, looks like this is the recipe:
  // note that fout is access via extern from zmodem_sz.cpp, so must stick
  // with that name.
  
  // Start the ZMODEM transfer
  Filesleft = 1; // these are globals from zmodem_sz.cpp
  Totalleft = fout.fileSize();
  mySerial.print("rz\r");
  sendzrqinit();
  delay(200);
  wcs(filename);
  saybibi();
  fout.close();
}
#endif // HAS_ZMODEM

#ifdef HAS_XMODEM
#include "XModem.h"

void Storage::xmodem_send_file(const char *filename, xmodem_proto_t proto) {
  close_file(); // be sure we don't have some log file sitting open.
  
  if (!myFile.open(filename, O_READ)) {
    mySerial.println("file.open failed");
    return;
  }
  if(proto==XMODEM)
    mySerial.write("rx\r"); // ad-hoc trigger for download
  else
    mySerial.write("rb\r"); // ad-hoc trigger for download

  char mode;
  if (proto==XMODEM) {
    mode=ModeXModem;
  } else {
    mode=ModeYModem;
  }
  XModem xmodem(&mySerial, mode);
  
  xmodem.sendFile(myFile,filename);
  close_file();
}
#endif // HAS_XMODEM
      
void Storage::help() {
  mySerial.println("  Storage");
  mySerial.println("    erase       # erase SD card");
  mySerial.println("    format      # format SD card");
  mySerial.println("    quickformat # as advertised");
  mySerial.println("    ls          # show files on sd card");
  mySerial.println("    sd_status   # display SD info");
  mySerial.println("    sd_open     # open next file for output");
  mySerial.println("    sd_close    # close current output file");
#ifdef HAS_ZMODEM
  mySerial.println("    sz=filename.ext # send file via zmodem");
#endif
#ifdef HAS_XMODEM
  mySerial.println("    sx=filename.ext # send file via xmodem");
  mySerial.println("    sb=filename.ext # send file via ymodem");
#endif
}
