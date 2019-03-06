#ifndef GPS_H
#define GPS_H

#include "Sensor.h"

#define IGNORE_NMEA_GPGSV
#define IGNORE_NMEA_GPGGA
#define IGNORE_NMEA_GPGLL
#define IGNORE_NMEA_GPVTG

// mtk3339 outputs $GPRMC, $GPVTG, $GPGGA, $GPGSA
#define MAX_NMEA_SENTENCES 4
// per the standard
#define MAX_SENTENCE_LENGTH 82

class GPS : public Sensor {
 public:
  // ring buffer
  // sentence being recorded
  volatile int write_sentence;
  // next character to be recorded
  int write_char;
  // next sentence to be logged
  volatile int read_sentence;

  char sentence_buff[MAX_NMEA_SENTENCES][MAX_SENTENCE_LENGTH];

  GPS() {
    strcpy(name,"gps");
    write_sentence=0;
    write_char=0;
    read_sentence=0;
  }

  virtual void init(void);
  virtual void async_read(void);
  void watch(void);
  void record_byte(char c);
  void serial_callback();

  virtual bool dispatch_command(const char *cmd, const char *cmd_arg);
  virtual void help();
  virtual void write_frame_info(Print &out);
  virtual void write_data(Print &out);

};



#endif // GPS_H
