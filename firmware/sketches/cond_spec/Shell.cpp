#include <Arduino.h>
#include <SdFat.h>

#include "Shell.h"


void Shell::setup(void) {
  mode = MODE_BOOT;
  request_mode = MODE_COMMAND;

  Serial.begin(115200);
  while( !Serial ) ;

  pinMode(LED_BUILTIN,OUTPUT);
  for(int i=0;i<3;i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  Serial.println("# Hello");
}

void Shell::loop(void) {
  // freebird included mode logic here
  command_loop();
}

void Shell::command_loop(void) {
  get_next_command("seaduck>\r\n");

  dispatch_command();
}

void Shell::dispatch_command(void) {
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
    SCB_AIRCR = 0x5FA0004;
    delay(100); // restart has a small lag
    // never gets here
  } else {
    Serial.println("\nUNKNOWN_COMMAND");
    Serial.println(cmd);
  }
}

bool Shell::activate_cmd_file(const char *fname) {
  // Check if file exists
  SdFile file; // Should this be SdFile ??
  if ( file.open(fname,O_READ) ) {
    Serial.print("Found cmd file ");
    Serial.println(fname);
    strcpy(cmd_filename,fname);
    cmd_file_pos=0;
    return true;
  } else {
    Serial.print("Failed to activate command file ");
    Serial.println(fname);
    
    cmd_filename[0]='\0';
    cmd_file_pos=0;
    return false;
  }
}

void Shell::get_next_command(const char *prompt) {
  //  show prompt and then wait for a complete
  //  line of input.

  bool use_serial=(!cmd_filename[0]);
  
  SdFile file;
  
  uint8_t pos=0;
  int16_t tmp;

  if( !use_serial ) {
    if ( !file.open(cmd_filename,O_READ) ) {
      Serial.print("Failed to open cmd file");
      Serial.println(cmd_filename);
      use_serial=true;
    } else {
      file.seekSet(cmd_file_pos);
    }
  }
  
  while( pos==0 ) {
    // clear buffer before showing prompt
    if ( use_serial ) {
      while( Serial.available() ) Serial.read();
      Serial.print(prompt);
    }

    cmd_arg=NULL;
    
    while(pos<CMD_BUFFLEN-1) {
      if( use_serial ) {
        while(!Serial.available()) ;
        cmd[pos] = Serial.read();
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
          cmd[pos] = (uint8_t)tmp;
        }
      }
      
      // handle backspace, lamely.
      if( cmd[pos] == 8 ) {
        Serial.print(" CANCEL");
        pos = 0; // signal restart
        break;
      } else {
        if( cmd[pos] == '\r' or cmd[pos] == '\n' ) {
          // a little tricky, since it's possible to get cr, cr/lf, or lf
          // newlines.  This could be left over from a previous line.
          // for console use, better to print the prompt again so that
          // on connect, you can just hit enter a few times to make sure
          // there is a connection.
          if( use_serial )
            Serial.println();
          cmd[pos] = '\0'; // null terminate it
          break;
        }
        if( use_serial )
          Serial.print(cmd[pos]); // echo back to user

        // special handling for separate =
        if( cmd[pos]=='=' ) {
          cmd[pos] = '\0';
          cmd_arg=cmd+pos+1;
        }
        pos++;
        if(pos==CMD_BUFFLEN) { // protect overrun:
          Serial.print(" TOO_LONG");
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
  Serial.println("Available commands:");
  Serial.println("  System");
  Serial.println("    softboot # reboot");
}
