#ifndef RTCLOCK_H
#define RTCLOCK_H

class RTClock {
public:
  void init();
  void read();

  bool dispatch_command(const char *cmd, const char *cmd_arg);
  void help();
  
  // unsigned long reading_seconds;
  time_t reading_seconds;
  unsigned int reading_partial;

  void status();
  void watch();
};

#endif // RTCLOCK_H
