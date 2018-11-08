#include "samd_IntervalTimer.h"

// statically track how many timers are created
static volatile int samd_timer_count=0;

//define the interrupt handlers
// these must match whatever is in the constructor
void TC3_Handler(){
  Adafruit_ZeroTimer::timerHandler(3);
}
void TC4_Handler(){
  Adafruit_ZeroTimer::timerHandler(4);
}

SAMD_IntervalTimer::SAMD_IntervalTimer(uint8_t tn)
  : Adafruit_ZeroTimer(tn) {
  samd_timer_count++;
  // can't print stuff here.
}

// trim it down to the most direct signature
bool SAMD_IntervalTimer::begin(void (*funct)(), unsigned int period_us)
{
  unsigned pulseWidth_us=period_us/2; // default 50% duty cycle
  unsigned periodCounter, PWcounter;
  tc_clock_prescaler prescale;
  
  if (period_us == 0 ) return false;

  // straight out of https://github.com/avandalen/avdweb_SAMDtimer/blob/master/avdweb_SAMDtimer.cpp

  // ticks per period
  periodCounter = (F_CPU * (signed)period_us) / 1000000; // why signed?
  // ticks per pulse on-cycle
  PWcounter = (F_CPU * (signed)pulseWidth_us) / 1000000;

  // this is all assuming a 16 bit timer width
  if(periodCounter < 65536) prescale = TC_CLOCK_PRESCALER_DIV1; 
  else if((PWcounter >>= 1, periodCounter >>= 1) < 65536) prescale = TC_CLOCK_PRESCALER_DIV2; // = 256
  else if((PWcounter >>= 1, periodCounter >>= 1) < 65536) prescale = TC_CLOCK_PRESCALER_DIV4; 
  else if((PWcounter >>= 1, periodCounter >>= 1) < 65536) prescale = TC_CLOCK_PRESCALER_DIV8; 
  else if((PWcounter >>= 1, periodCounter >>= 1) < 65536) prescale = TC_CLOCK_PRESCALER_DIV16; 
  else if((PWcounter >>= 2, periodCounter >>= 2) < 65536) prescale = TC_CLOCK_PRESCALER_DIV64; 
  else if((PWcounter >>= 2, periodCounter >>= 2) < 65536) prescale = TC_CLOCK_PRESCALER_DIV256; 
  else if((PWcounter >>= 2, periodCounter >>= 2) < 65536) prescale = TC_CLOCK_PRESCALER_DIV1024; 

  /********************* Timer #4, 8 bit, one callback with adjustable period */
  /* based on https://github.com/avandalen/avdweb_SAMDtimer/blob/master/avdweb_SAMDtimer.cpp
      - 16 bit timer => max period 1.4s.
      - they use 16 bit timer, and channel 1 (because channel 0 has other
        duties in 16 and 32 bit timers.)
  */
  configure(prescale, 
            TC_COUNTER_SIZE_16BIT,   // bit width of timer/counter -- code above assumes 16
            TC_WAVE_GENERATION_MATCH_PWM  // regular counter, I think.
            );
  setPeriodMatch(periodCounter, PWcounter, 1); // 1 match, channel 1

  // these probably okay
  setCallback(true, TC_CALLBACK_CC_CHANNEL1, funct);
  enable(true);
  return true;
}

void SAMD_IntervalTimer::end(void)
{
  enable(false);  // yes?
}

