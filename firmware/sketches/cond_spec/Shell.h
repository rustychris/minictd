#ifndef SHELL_H
#define SHELL_H

#define CMD_BUFFLEN 100
#define LABEL_BUFFLEN 100

#define MODE_BOOT 0
#define MODE_SAMPLE 1
#define MODE_COMMAND 2

class Shell {
protected:
  char label[LABEL_BUFFLEN];
  char cmd[CMD_BUFFLEN];
  // when a command with an '=' is found, it's split there
  // with the second part pointed to by cmd_arg
  char *cmd_arg;
  uint8_t request_mode,mode;

public:
  Shell(void) {} ;

  void setup(void);

  void loop(void);

  void command_setup(void) {};
  void command_loop(void);
  void command_cleanup(void) {};

  // if there is an active command file, return the next
  // command from it, otherwise, prompt for serial input.
  void get_next_command(const char *);
  virtual void help(void);

  virtual void dispatch_command(void);
};


#endif // SHELL_H
