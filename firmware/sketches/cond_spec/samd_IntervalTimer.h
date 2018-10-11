#ifndef SAMD_INTERVALTIMER_H
#define SAMD_INTERVALTIMER_H

#include <Adafruit_ZeroTimer.h>

// wrap adafruit zero timer to have same api as IntervalTimer, at
// least at the level that I'm using it.
class SAMD_IntervalTimer : public Adafruit_ZeroTimer
{
 public:
  // start simple -- hardcode which timer.  it's possible based on zero timer
  // comments that only even-numbered timers can be used in 32-bits, which we
  // will probably want.

  bool begin(void (*funct)(void),unsigned int microseconds);

  void end(void);

  SAMD_IntervalTimer(); // : Adafruit_ZeroTimer(4) {;};

  ~SAMD_IntervalTimer() {
		end();
	}

  // unsigned period_us, periodCounter, PWcounter;
  // tc_clock_prescaler prescale;
  // tc_counter_size countersize;  
};


#endif // SAMD_INTERVALTIMER_H
