#ifndef PRESSURE_H
#define PRESSURE_H

class Pressure {
public:
  void init();
  void read();
  void display();
  
  bool dispatch_command(const char *cmd, const char *cmd_arg);
  void help();

  double pressure_baseline;
  
  double pressure_abs;
  float temperature_c;
  
  
};

#endif // PRESSURE_H
