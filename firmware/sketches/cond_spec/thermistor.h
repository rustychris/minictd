#ifndef THERMISTOR_H
#define THERMISTOR_H

class Thermistor {
public:
  void init();
  void read();

  bool dispatch_command(const char *cmd, const char *cmd_arg);
  void help();
  
  float reading;
};

#endif // THERMISTOR_H
