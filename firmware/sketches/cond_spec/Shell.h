#ifndef SHELL_H
#define SHELL_H

#define CMD_BUFFLEN 100
#define LABEL_BUFFLEN 100
#define MAX_FILENAME_LEN 256

#define MODE_BOOT 0
#define MODE_SAMPLE 1
#define MODE_COMMAND 2

// wasn't necessary on teensy, but maybe needed for feather
// unsure of whether this will break teensy build.
#include <inttypes.h>

class Shell {
protected:
  char label[LABEL_BUFFLEN];
  // base_cmd and base_cmd_arg are state that is updated at the top level
  // parsing (i.e. get_next_command).  Recursive calls to dispatch_command
  // are possible by passing some other pair of strings.
  char base_cmd[CMD_BUFFLEN];
  // when a command with an '=' is found, it's split there
  // with the second part pointed to by cmd_arg
  char *base_cmd_arg;
  
  uint8_t request_mode,mode;

  // For reading commands from a file:
  char cmd_filename[MAX_FILENAME_LEN];
  uint32_t cmd_file_pos;

  // ?bool sample_monitor;

  // uint8_t frame[MAX_FRAME_BYTES];
  // uint8_t frame_pos;

public:
  Shell(void) {
    cmd_filename[0]='\0';
  } ;

  void setup(void);

  void loop(void);

  void command_setup(void) {};
  void command_loop(void);
  void command_cleanup(void) {};

  // if fname exists, it will be activated as the source for
  // subsequent calls to get_next_command(), until end of file is
  // reached.  returns true if file was found.
  bool activate_cmd_file(const char *fname);
  
  // if there is an active command file, return the next
  // command from it, otherwise, prompt for serial input.
  void get_next_command(const char *);
  virtual void help(void);

  virtual void dispatch_command(const char *cmd, const char *cmd_arg);
};


#endif // SHELL_H
