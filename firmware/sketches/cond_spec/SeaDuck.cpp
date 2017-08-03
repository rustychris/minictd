// Now that PDB_CONFIG is completely defined locally, no need for
// Audio import, right?
// #include <Audio.h>

#include "SeaDuck.h"
#include "pressure.h"
#include "thermistor.h"
#include "conductivity.h"

ADC *adc=new ADC();

Conductivity cond;

void check_adc_error() {
  if ( adc->adc1->fail_flag != ADC_ERROR_CLEAR ) {
      Serial.print(" ADC1 error: ");
      Serial.println(adc->adc1->fail_flag);
      adc->adc1->fail_flag=ADC_ERROR_CLEAR;
      while(1);
  }
  if ( adc->adc0->fail_flag != ADC_ERROR_CLEAR ) {
      Serial.print(" ADC0 error: ");
      Serial.println(adc->adc0->fail_flag);
      adc->adc0->fail_flag=ADC_ERROR_CLEAR;
      while(1);
  }
}

/*
 * Configure aspects of the ADCs that will not change
 * with whether it's being used by conductivity or
 * thermistor
 */
void common_adc_init(void) {
  for(int adc_num=0;adc_num<2;adc_num++) {
    // used to be ADC_REF_3V3
    adc->setReference(ADC_REFERENCE::REF_3V3, adc_num);
    delay(5);
    adc->setResolution(16,adc_num);
    delay(5);
    adc->setAveraging(1,adc_num);
    adc->disablePGA(adc_num);
    delay(5);

    // default settings are good down to pdb period of 300.
    if(0) {
      // these settings are okay down to pdb period of 175.
      // adc->setConversionSpeed(ADC_HIGH_SPEED_16BITS,adc_num);
      adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS,adc_num);
      delay(5);
      adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED,adc_num);
      delay(5);
    } else {
      // probably ill-advised
      adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED,adc_num);
      delay(5);
      adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED,adc_num);
      delay(5);
    }
  }
  
  check_adc_error();
  // just to be sure that ADC trigger is from PDB.  This configures the source
  // of the trigger, but whether the ADC is automatically triggered or not
  // is configured elsewhere, so this is safe even if direct triggering for
  // the thermistor is being used
  SIM_SOPT7=0;
}


SeaDuck::SeaDuck() 
{
  pinMode(POWER_3V3_ENABLE_PIN,OUTPUT);
  digitalWrite(POWER_3V3_ENABLE_PIN,HIGH);

}


 
void SeaDuck::setup() {
  Shell::setup();
  
  common_adc_init();

  // cond_init();

  cond.setup();
  pressure_setup();
  ntc_setup();
}

void SeaDuck::dispatch_command() {
  if ( strcmp(cmd,"scan")==0 ) {
    scan();
  } else if ( strcmp(cmd,"scan_period")==0 ) {
    if(cmd_arg) {
      int32_t new_period=atoi(cmd_arg);
      uint8_t new_prescaler=0;
      if ( new_period < 100 ) {
        Serial.println("ERROR: period is too short (<100)");
      } else {
        while ( new_period > ((int32_t)1<<15) ) {
          new_period >>= 1;
          new_prescaler++;
        }
        if ( new_prescaler > 7 ) {
          Serial.println("ERROR: period is too long");
          pdb_prescaler=0;
          return;
        }
        Serial.print("new_period="); Serial.println(new_period);
        Serial.print("new_prescaler="); Serial.println(new_prescaler);
        pdb_prescaler=new_prescaler; 
        pdb_period=new_period;
      }
    } else {
      Serial.print("scan_period="); Serial.println( pdb_period*(1<<pdb_prescaler) );
    }
  } else if ( strcmp(cmd,"dac_oversample")==0) {
    if(cmd_arg) {
      int new_oversample=(int16_t)atoi(cmd_arg);
      if ( (new_oversample < 1)
           || (new_oversample>32 )
           || ( new_oversample & (new_oversample-1)) != 0 ) {
        Serial.println("ERROR: oversample must be power of 2, 1..16");
      } else {
        dac_per_adc=new_oversample;
      }
    } else {
      Serial.print("dac_oversample="); Serial.println(dac_per_adc);
    }
  } else if ( strcmp(cmd,"stride")==0) {
    if(cmd_arg) {
      int new_stride=(int16_t)atoi(cmd_arg);
      if ( (new_stride < 1)
           || (new_stride>64 )
           || ( new_stride & (new_stride-1)) != 0 ) {
        Serial.println("ERROR: stride must be power of 2, 1..64");
      } else {
        dac_out_stride=new_stride;
      }
    } else {
      Serial.print("stride="); Serial.println(dac_out_stride);
    }
  } else if (strcmp(cmd,"dac_status")==0) {
    dac_status();
  } else if ( strcmp(cmd,"n_discard")==0) {
    if(cmd_arg) {
      int new_discard=(int16_t)atoi(cmd_arg);
      if ( (new_discard < 0)
           || (new_discard>100 )) {
        Serial.println("ERROR: n_discard must be 0..100");
      } else {
        adc_n_discard=new_discard;
      }
    } else {
      Serial.print("n_discard="); Serial.println(adc_n_discard);
    }
  }
  // new way - set loop count directly.
  else if ( strcmp(cmd,"n_loops")==0 ) {
    if (cmd_arg) {
      int new_n_loops=(int16_t)atoi(cmd_arg);
      if ( (new_n_loops<1) ||
           (new_n_loops>1000) ) {
        // actually 1000 is arbitrary, but to avoid bugs leading
        // to an apparent hang, keep it reasonably low
        Serial.println("ERROR: n_loop must be between 1 and 1000");
      } else {
        adc_n=new_n_loops;
        adc_var_shift=0;
        while( (4<<adc_var_shift) < adc_n ) adc_var_shift++;
      }
    } else {
      Serial.print("n_loops="); Serial.println(adc_n);
    }
  }
  else if ( strcmp(cmd,"dac_shift")==0) {
    if(cmd_arg) {
      int new_shift=(uint16_t)atoi(cmd_arg);
      if ( (new_shift < 0) || (new_shift>15 ) ) {
        Serial.println("ERROR: shift must be between 0 and 15");
      } else {
        dac_shift=new_shift;
      }
    } else {
      Serial.print("dac_shift="); Serial.println(dac_shift);
    }
  } else if ( strcmp(cmd,"dac_mid")==0) {
    if(cmd_arg) {
      uint16_t new_mid=(uint16_t)atoi(cmd_arg);
      if ( (new_mid < 0) || (new_mid>4095 ) ) {
        Serial.println("ERROR: dac output range is 0..4095");
      } else {
        dac_mid=dac_mid;
      }
    } else {
      Serial.print("dac_mid="); Serial.println(dac_mid);
    }
  } else if ( strcmp(cmd,"pressure") ) {
    pressure_read();
  } else if ( strcmp(cmd,"temperature") ) {
    ntc_read();
  } else {
    Shell::dispatch_command();
  }
}

void SeaDuck::help() {
  Shell::help();
  Serial.println(" scan  # run initiate a transfer function scan");
  Serial.println(" scan_period[=NNNN] # set the speed of the scan");
  Serial.println(" dac_stride[=1,2,4,8,...] # shorten waveform");
  Serial.println(" dac_oversample[=1,2,4,8,16] # oversample DAC");
  Serial.println(" accum_shift[=0..6] # average over 4*2^N loops");
  Serial.println(" dac_shift[=0..15] # scaling of wave table");
  Serial.println(" dac_mid[=0..4095] # center point for dac output");
  Serial.println(" n_loops[=1..1000] # number of averaging loops");
  Serial.println(" dac_status # diagnostic printout ");
}




// Is it possible to give the DAC DMA priority?
// It has to run faster, and with tighter constraints
// than the ADC DMAs.  None of the obvious changes related
// to priority helped.

// DMAPriorityOrder(dac_dma, adc0_dma, adc1_dma); // ??
// That did nothing.

// Running out of options...

//  Drop the ISR's?  
//  Any better if adc1 is not part of the DMA? 
// Removed adc1 entirely, no better.

// It's possible that with the noisy power, a really short interrupt signal is
// getting lost?  different Teensy, on usb power, shows similar behavior
// but not quite as bad.  still, USB power should be that bad.  have to rule
// out power here.

// What about making some pulses longer by choosing smaller pdb_mod, larger
// prescaler?  That made it much worse - 21 instead of 5 or 6.
// Weird - so it's not about the length of each PDB cycle, it's about how 
// much is going on or could go on within a single PDB clock.

// having the ramp lets us read the dac index directly.  that 

// Have a second DMA which is triggered off the PDB, and just shuffles
// through the DAC buffer.

// oversample=1
// even down to 512, and 80 loops, no errors.

// scan_period vs. dac error after "40" (really 20) loops:
// oversample=2
// 4096 => 2
// 2048 => 5
// 1024 => 10  [see below]
// 512  => 21
// 256  => 43

// oversample=4
// scan_period
// 4096 => 15
// 2048 => 30
// 1024 => 60

// Weird.
// presumably it is only the dac advances during the middle
// of the PDB which are getting lost.  That would explain why with
// oversample=1, there are no errors.  But if the middle dac advances
// are prone to a nominal failure rate, then shouldn't it be a factor
// of 3 increase from oversample=2 to 4?
// instead it was a factor of 6.

// at oversample=8
// 8192 => 38
// 4096 => 72, 73
// 2048 => 146

// doesn't matter if dac_status() is turned off during loop, or
// if delay is 10 vs. 20 ms.

// It's possible, though iffy, that the ADC is overactive.  In fact,
// that could explain the factor of 6-ish.  Say the ADC has some
// chance of being advanced incorrectly by any dac advance in the
// middle of the pdb (i.e. the dacint trigger sometimes flips the
// adc trigger, too).
//  so dac_oversample=1 is safe because something is different at the end.
//  dac_oversample=2 has issues, and some fraction of the time advances the adc.
//  take that fraction to be 100%
//    each false trigger creates an error which is then multiplied by 2. one loop
//    introduces 1 extra ADC, yielding an error of 2
//  dac_oversample=4 has 3 chances to flip the adc.  each flip increases the error
//  4, so total errors are 12, or ** 6 ** times dac_oversample=2.
//  dac_oversample=8 has 7 chance to flip the adc.  each flip increases the error
//  by 8, so total errors are 56, or 4.67 times dac_oversample=4.  The 15 errors
//  at dac_oversample=4 / scan_period=4096 become 70 errors at dac_oversample=8,
//  close enough to the 72/73 observed.

// why would there be a PDB_MOD dependence, though???

// deconstructing the numbers one step further -
// 20 ADC loops at scan_period=1024 and dac_oversample=2 yields
// 10 errors.  That's 5 times that the ADC was erroneously triggered.
// If there is an even chance of the ADC triggering on every non-end
// DAC trigger, how many times did the chance present itself?
// ADC looped 20 times, so DAC looped 40 times, or 40*256 samples.
// half of those were at the end, so there were 20*256 chance for the
// DAC to cause a problem.  of those 20*256, it happened 5 times, or
// once per ... 1024.  which just happens to be PDB_MOD.

// but that's saying that for each DAC trigger, there is a 1/1024 chance
// that it fires the ADC, too.  

// One bit of evidence difficult to fit into the puzzle is that it seemed
// to work okay when the last dac interval had some space before the end
// of the PDB_MOD.  i.e. PDB_MOD=1027, DACINT0=256.

// still confusing.

// Two new things to try:
// For the ADC DMA - copy from PDB_CNT instead to see if it ever
// is triggered outside the expected time.
//  This caused a lot of havoc, doesn't seem like we can do that...

// Other thing is to watch the PDB error register
// in the base case: dac_oversample=2, n_loops=40, scan_period=1024,
// we get index error of 10, and no sequence errors

// an aside - try some wacky values for ch1 delay
// 0: nope.

// Maybe we have to read the ADC in order for it to be okay
// with the next conversion.


// A second option is to copy the DAC read pointer, to get
// a modulo-16 idea of the loop index.


// is it possible that pdb_mod should be 1 less?
// now it gets some errors even for dac_oversample=1.

// shortening both period and dac_int by 1 worked for dac_oversample=2,
// but not for dac_oversample=4.

// do i need SIM_SOPT7 = 0; ? not really.

// standard case, but n_loops=2 -
// I get a weird 603,617,602,617 repeated pattern in counter.
// seems to happen regardless of dacint.
// okay - this is the round-robin scheduling for DMA.
// stabilized with DMAPriority...
// deviations from 147+-1:
// last loop of 20 -
// sample 130 was at 154
// dac_oversample=4:
// deviations:
//  234: 162
//

// one loop, dac_oversample=4
//  error=6
//  213: 156
//  seems more like a bit of a DMA priority

// Hard to call this a real issue.
// Looking at the DAC read pointer -
// all over the map.
// idx, dac read ptr
// 0,14  // not sure why it starts on 14
//    2
//    6
//   10
//   14
// at output 50, instead of 6 it shows 5 in the DAC.
// then it's 5,9,13,1...
// then at 136, it again drops a sample, going from 9 to
// 12 instead of 13.
// 222 drops another - going from 0 to 3
// So that's 3 DAC samples apparently lost.  The 6 number
// comes after the PDB runs a bit longer.

// trying again: starts at 14 again
// at 50, goes to 5 instead of 6
// at 136, goes to 12 instead of 13
// at 222, goes to 3 instead of 4
// same as before.

// So there is something special about these
// when dac_oversample=4, it starts at 14, when 2, starts at 0

// on a lark, changing the order of the dma object declarations.
// no change.

// So on select times through the PDB loop, the DAC loses
// samples.
// this is somehow related to the number of samples the DAC advances
// not including the end.
// it appears that the issue is *not* the ADC skipping ahead.  In that
// case, we would see a repeat of the DAC index, or maybe an intermediate
// number.

// In the case above:
// We can ignore where the DAC starts (I think).
// The ADC DMA reliably occurs at 147 pdb counts +-1.
// on iteration 50, pdb count 147, let's say the
// DAC interval counter was at 255, one shy of its dacint.
// 86 iterations later, at iteration 136, when the pdb count
// was at 147, we lost another DAC, so let's say that it again
// was at dacinterval=255.  And had "lost" 256 counts over those
// 86 iterations.
// another 86 iterations, and we have lost yet another dac sample.

// so the dac interval counter is losing 256 counts per 86 iterations,
// or very nearly 3 counts per pdb iteration.

// one pdb iteration is supposed to be 1024 counts long
// okay - maybe this is why adding 3 pdb counts to the loop fixed it
// right - when we did 1027 instead of 1024 it worked.
// I'm not 100% sure that was with the dac_oversample=4 case.

// But my fear is that this behavior is affected by some the pdb-period

// I can run the same test, but with other pdb_mod, and other dac_oversample,
// to get a sense in those other cases of how often it loses a dac sample.
// it may require looking at the scope I think to find out whether the problem
// is evenly distributed over the pdb iteration.  possibly could be done by changing
// the delay of the ADC conversion, although that may alter the problem, too.

// with pdb_period=1024, n_loops=1, dac_oversample=2, the rdptr shows
// no lost samples.  but n_loops=2, then in the second loop, at index
// 148, it drops one dac sample. no samples dropped on third loop...
// fourth loop at sample 150 is drops one.  So the time between dropping dac
// samples is (second half of loop 2, loop 3, first half of loop 4).
// or 512 adc samples, 512 times through the pdb loop.  So each time through
// the pdb loop we are off by one dacinterval count.  dac_int=512, so after
// 512 pdb loops, we lose one dac sample.  in this case, the correction would be
// to make the pdb_loop 1 count longer, i.e. dac_sample-1.

// with pdb_period=1024, n_loops=1, dac_oversample=8...
// drops one at sample 5, then 24.  so dropped sample in 19 iterations!?
// then 42, so 18 iterations.
// 18 pdb loops.  dac_interval is 128, 128 counts / (18 pdb loops) =
// 7.1 counts per loop.  yep.  we lose dac_oversample-1 counts per loop,
// so again, pdb_interval should be increased by (dac_oversample-1)


// It seems like this might be solved, but there is still to-do some testing with
// the actual circuit to see if the DAC and ADC continue to have offsets, and
// to also check the DAC signal on the scope for signs of creep.

// At that point, can collect some spectra and try some fits
