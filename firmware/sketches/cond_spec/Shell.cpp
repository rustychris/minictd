#include <Arduino.h>
#include <SdFat.h>
#include <elapsedMillis.h>

#include "Shell.h"
#include "serialmux.h"
#include <AWire.h>

void Shell::setup(void) {
  mode = MODE_BOOT;
  request_mode = MODE_COMMAND;

  mySerial.begin(115200);
  elapsedMillis elapsed;
  // for debugging, it's nice to see the start
  // of everything, so wait a few seconds.
  // then plow ahead so we can also be functional
  // headless.
  while( (!mySerial) && (elapsed<3000) );

  mySerial.println("# Hello");
}

void Shell::loop(void) {
  // freebird included mode logic here
  command_loop();
}

void Shell::command_loop(void) {
  get_next_command("seaduck>\r\n");

  dispatch_command(base_cmd,base_cmd_arg);
}

void Shell::dispatch_command(const char *cmd, const char *cmd_arg) {
  if ( strcmp(cmd,"help")==0 ) {
    help();
  } else if ( strcmp(cmd,"!")==0 ) {
    // do nothing - this is just to get to command mode.
  } else if ( strcmp(cmd,"softboot")==0 ) {
    // ugly...
    // from http://forum.pjrc.com/threads/24304-_reboot_Teensyduino()-vs-_restart_Teensyduino()
    // but adapted to use registers from mk20dx128.h
    // not sure what the exact meaning of the bits are.
    // note that this DOES reset USB.
#ifdef CORE_TEENSY
    SCB_AIRCR = 0x5FA0004;
#else
    // M0 reset: https://forum.arduino.cc/index.php?topic=379950.0
    // seems that it will just hang if we don't stop Serial first.
    // not sure which of these are required, but at least one of them
    // is.
    Serial.end();
#ifdef BT2S
    BT2S.end();
#endif
    NVIC_SystemReset();
#endif
    delay(100); // restart has a small lag
    // never gets here
  } else {
    mySerial.println("\nUNKNOWN_COMMAND");
    mySerial.println(cmd);
  }
}

bool Shell::activate_cmd_file(const char *fname) {
  // Check if file exists
  SdFile file; // Should this be SdFile ??
  if ( file.open(fname,O_READ) ) {
    mySerial.print("# Found cmd file ");
    mySerial.println(fname);
    strcpy(cmd_filename,fname);
    cmd_file_pos=0;
    return true;
  } else {
    mySerial.print("# Command file not activated: ");
    mySerial.println(fname);
    
    cmd_filename[0]='\0';
    cmd_file_pos=0;
    return false;
  }
}

void Shell::get_next_command(const char *prompt) {
  //  show prompt and then wait for a complete
  //  line of input.

  // TODO: allow # comment character in first column
  
  bool use_serial=(!cmd_filename[0]);
  
  SdFile file;
  
  uint8_t pos=0;
  int16_t tmp;

  if( !use_serial ) {
    if ( !file.open(cmd_filename,O_READ) ) {
      mySerial.print("Failed to open cmd file");
      mySerial.println(cmd_filename);
      use_serial=true;
    } else {
      file.seekSet(cmd_file_pos);
    }
  }
  
  while( pos==0 ) {
    // clear buffer before showing prompt
    if ( use_serial ) {
      while( mySerial.available() ) mySerial.read();
      mySerial.print(prompt);
    }

    base_cmd_arg=NULL;
    
    while(pos<CMD_BUFFLEN-1) {
      if( use_serial ) {
        while(!mySerial.available()) ;
        base_cmd[pos] = mySerial.read();
      } else {
        tmp=file.read();
        if( tmp<0 ) {
          // error or end of file.
          cmd_filename[0]='\0';
          pos=0; // signal restart with prompt
          use_serial=true;
          // there is a timing issue with printing a message
          // about this event here.
          break;
        } else {
          base_cmd[pos] = (uint8_t)tmp;
        }
      }
      
      // handle backspace, lamely.
      if( base_cmd[pos] == 8 ) {
        mySerial.print(" CANCEL");
        pos = 0; // signal restart
        break;
      } else {
        if( base_cmd[pos] == '\r' or base_cmd[pos] == '\n' ) {
          // a little tricky, since it's possible to get cr, cr/lf, or lf
          // newlines.  This could be left over from a previous line.
          // for console use, better to print the prompt again so that
          // on connect, you can just hit enter a few times to make sure
          // there is a connection.
          if( use_serial )
            mySerial.println();
          base_cmd[pos] = '\0'; // null terminate it
          break;
        }
        if( use_serial )
          mySerial.print(base_cmd[pos]); // echo back to user

        // special handling for separate =
        if( base_cmd[pos]=='=' ) {
          base_cmd[pos] = '\0';
          base_cmd_arg=base_cmd+pos+1;
        }
        pos++;
        if(pos==CMD_BUFFLEN) { // protect overrun:
          mySerial.print(" TOO_LONG");
          pos = 0; // signal restart
          break;
        }
      }
    }
  }
  if( !use_serial )
    cmd_file_pos=file.curPosition();
}

void Shell::help(void){
  mySerial.println("Available commands:");
  mySerial.println("  System");
  mySerial.println("    softboot # reboot");
}
