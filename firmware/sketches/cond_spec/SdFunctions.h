#ifndef __SDFUNCTIONS_H__
#define __SDFUNCTIONS_H__

// options for c
//  'E' erase only
//  'F' erase and format
//  'Q' quick format only

#include <Print.h>

#ifdef HAS_XMODEM
typedef enum {
  XMODEM=0,
  YMODEM=1,
} xmodem_proto_t;
#endif


class Storage : public Print {
public:
  enum SdStatus {DISABLED,NOCARD,ENABLED};
  // may not be necessary - debuging Serial vs Storage issue
  volatile SdStatus status;

  Storage(void) { sync_interval_blocks=30; }

  virtual size_t write(uint8_t b);

  // called once on system startup
  void init(void);

  void format(char c);

  void help(void);
  bool dispatch_command(const char *, const char *);
  
  // to be called from the sample loop to handle periodic writes
  void loop(void);
  void cleanup(void);

  void info(void);

  void open_block();
  void close_block(void);

  // DATAnnnn.BIN
  char active_filename[13];
  void set_next_active_filename(void);
  void open_next_file(void);
  void close_file(void);

#ifdef HAS_ZMODEM
  void zmodem_send_file(const char *filename);
#endif
#ifdef HAS_XMODEM
  void xmodem_send_file(const char *filename,xmodem_proto_t proto);
#endif
  
  // incremented by the number of samples the
  // ISR couldn't write because there were no more free buffer blocks.
  uint32_t overruns;
  
  // when to call sync:
  uint16_t sync_interval_blocks,sync_counter;
};

#endif // __SDFUNCTIONS_H__
