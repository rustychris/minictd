// Now that PDB_CONFIG is completely defined locally, no need for
// Audio import, right?
// #include <Audio.h>

#include "SeaDuck.h"

// since ADC and Audio define this differently, define our own here.
// the only real difference is DMAEN.  That means that PDB triggers
// a dma transfer on looping, rather than an interrupt.
// TRGSEL(15): software trigger
// dropped PDBIE and DMAEN as they are no longer needed for the DAC.
#define LOC_PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_CONT  \
                        | PDB_SC_LDMOD(0) )

ADC *adc=new ADC();

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

// borrow a sine wave table from data_waveforms.c in the Audio lib.
// this could be created at startup in the dac_buffer for some
// flexibility
const int16_t AudioWaveformSine[257] = {
     0,   804,  1608,  2410,  3212,  4011,  4808,  5602,  6393,  7179,
  7962,  8739,  9512, 10278, 11039, 11793, 12539, 13279, 14010, 14732,
 15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403,
 22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
 27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571,
 30852, 31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521,
 32609, 32678, 32728, 32757, 32767, 32757, 32728, 32678, 32609, 32521,
 32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
 30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683, 27245, 26790,
 26319, 25832, 25329, 24811, 24279, 23731, 23170, 22594, 22005, 21403,
 20787, 20159, 19519, 18868, 18204, 17530, 16846, 16151, 15446, 14732,
 14010, 13279, 12539, 11793, 11039, 10278,  9512,  8739,  7962,  7179,
  6393,  5602,  4808,  4011,  3212,  2410,  1608,   804,     0,  -804,
 -1608, -2410, -3212, -4011, -4808, -5602, -6393, -7179, -7962, -8739,
 -9512,-10278,-11039,-11793,-12539,-13279,-14010,-14732,-15446,-16151,
-16846,-17530,-18204,-18868,-19519,-20159,-20787,-21403,-22005,-22594,
-23170,-23731,-24279,-24811,-25329,-25832,-26319,-26790,-27245,-27683,
-28105,-28510,-28898,-29268,-29621,-29956,-30273,-30571,-30852,-31113,
-31356,-31580,-31785,-31971,-32137,-32285,-32412,-32521,-32609,-32678,
-32728,-32757,-32767,-32757,-32728,-32678,-32609,-32521,-32412,-32285,
-32137,-31971,-31785,-31580,-31356,-31113,-30852,-30571,-30273,-29956,
-29621,-29268,-28898,-28510,-28105,-27683,-27245,-26790,-26319,-25832,
-25329,-24811,-24279,-23731,-23170,-22594,-22005,-21403,-20787,-20159,
-19519,-18868,-18204,-17530,-16846,-16151,-15446,-14732,-14010,-13279,
-12539,-11793,-11039,-10278, -9512, -8739, -7962, -7179, -6393, -5602,
 -4808, -4011, -3212, -2410, -1608,  -804,     0
};

#define BLOCKSIZE 256
DMAMEM static uint16_t dac_buffer[BLOCKSIZE];
DMAMEM static uint16_t adc0_buffer[BLOCKSIZE];
DMAMEM static uint16_t adc1_buffer[BLOCKSIZE];
DMAChannel dac_dma(true);
DMAChannel adc0_dma(true);
DMAChannel adc1_dma(true);

// Conductivity sensor:
int16_t pdb_period; // limited to 16 bits
uint8_t pdb_prescaler=0; // an additional 3 bit number for 2^(prescale)

// pseudo oversampling.  Each pdb loop triggers one ADC read,
// and this many dac writes.
int dac_per_adc=1;
int dac_out_stride=1; // new implementation, goes with buff_n_samples
int buff_n_samples=BLOCKSIZE / dac_out_stride; // for using partial buffers at high frequency.

// Keeping track of averaging:
volatile int adc0_n=0;
volatile int adc1_n=0;

// variance will be right shifted this amount - effectively makes
// 4<<adc_var_shift the max number of loops
int adc_n; // defines the number of loops
int adc_n_discard=0; // don't record the first n_discard times through
int adc_var_shift=4;

uint16_t dac_mid=1<<11;
uint16_t dac_shift=6;

int32_t adc0_accum[BLOCKSIZE];
uint32_t adc0_variance[BLOCKSIZE];
int32_t adc1_accum[BLOCKSIZE];
uint32_t adc1_variance[BLOCKSIZE];

SeaDuck::SeaDuck() 
{
  // set some basic, sane settings
  pdb_period=1025;
  dac_per_adc=2;
  adc_var_shift=4;
  adc_n=20;

  pinMode(POWER_3V3_ENABLE_PIN,OUTPUT);
  digitalWrite(POWER_3V3_ENABLE_PIN,HIGH);
}

void SeaDuck::fill_sine_buffer() {
  uint16_t i;
  buff_n_samples=BLOCKSIZE/dac_out_stride;
  for(i=0;i<buff_n_samples;i++) {
    // for debugging - write a ramp 
    // dac_buffer[i] = i;
    // writing into 16 bits, but value needs to be 0--4095
    dac_buffer[i] = (uint16_t) ( dac_mid + (AudioWaveformSine[i*dac_out_stride] >> dac_shift) );

    adc0_buffer[i] = 0;
    adc0_accum[i]=0;
    adc0_variance[i]=0;
    
    adc1_buffer[i] = 0;
    adc1_accum[i]=0;
    adc1_variance[i]=0;
  }
  // maybe helps with some debugging - zero out unused buffer portion.
  for(i=buff_n_samples;i<BLOCKSIZE;i++) {
    dac_buffer[i]=0;
    adc0_buffer[i]=0;
    adc1_buffer[i]=0;
    adc0_accum[i]=0;
    adc1_accum[i]=0;
    adc0_variance[i]=0;
    adc1_variance[i]=0;
  }
  adc0_n=0;
  adc1_n=0;
}

void adc0_dma_isr(void)
{
  int j,j_start,j_end;
  
  uint32_t daddr;
  daddr = (uint32_t)(adc0_dma.TCD->DADDR);
  adc0_dma.clearInterrupt();

  // maxed out - no more space to accumulate
  if (adc0_n >= adc_n ) return;
  
  // Following audio library:
  // buff_n_samples/2 used to be ( sizeof(adc0_buffer) / dac_out_stride / 2 )
  if (daddr < (uint32_t)(&adc0_buffer[buff_n_samples/2]) ) {
    adc0_n++; // signal that we'll end with a whole number of cycles
    if ( adc0_n <= adc_n_discard )
      return; // throwaway
    
    // DMA is receiving to the first half of the buffer
    // need to remove data from the second half
    j_start=buff_n_samples/2; //   BLOCKSIZE/2/dac_out_stride;
    j_end=buff_n_samples; // BLOCKSIZE/dac_out_stride;
  } else {
    // throwaway
    if ( adc0_n < adc_n_discard )
      return;
    
    // DMA is receiving to the second half of the buffer
    // need to remove data from the first half
    j_start=0;
    j_end=buff_n_samples/2; // BLOCKSIZE/2/dac_out_stride;
  }
  
  for(j=j_start;j<j_end;j++) {
    int16_t sample=(int16_t)adc0_buffer[j];
    adc0_accum[j] += sample;
    adc0_variance[j] += (sample*sample)>>adc_var_shift;
  } 
}

void SeaDuck::adc_init(void) {
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
  SIM_SOPT7=0; // just to be sure that ADC trigger is from PDB.
}

void adc1_dma_isr(void)
{
  int j,j_start,j_end;
  
  uint32_t daddr;
  daddr = (uint32_t)(adc1_dma.TCD->DADDR);
  adc1_dma.clearInterrupt();

  // maxed out - no more space to accumulate
  if (adc1_n >= adc_n ) return;
  
  // Following audio library:
  // sizeof(adc1_buffer)/dac_out_stride / 2
  if (daddr < (uint32_t)(&adc1_buffer[buff_n_samples/2]) ) {
    adc1_n++; // signal that we'll end with a whole number of cycles
    if (adc1_n<=adc_n_discard)
      return; // throwaway
    
    // DMA is receiving to the first half of the buffer
    // need to remove data from the second half
    j_start=buff_n_samples/2; // BLOCKSIZE/2/dac_out_stride;
    j_end=buff_n_samples; // /BLOCKSIZE/dac_out_stride;

  } else {
    if ( adc1_n < adc_n_discard )
      return; // throwaway
    
    // DMA is receiving to the second half of the buffer
    // need to remove data from the first half
    j_start=0;
    j_end=buff_n_samples/2; // BLOCKSIZE/2/dac_out_stride;
  }
  for(j=j_start;j<j_end;j++) {
    int16_t sample=(int16_t)adc1_buffer[j];
    adc1_accum[j] += sample;
    adc1_variance[j] += (sample*sample)>>adc_var_shift;
  } 
}

void SeaDuck::adc_setup(void) {
  adc0_setup();
  adc1_setup();
}

void SeaDuck::adc0_setup(void) {
  // Configure the ADC and run at least one software-triggered
  // conversion.  This completes the self calibration stuff and
  // leaves the ADC in a state that's mostly ready to use
  ADC0_SC1A = 0 | ADC_SC1_DIFF; 

  // enable the ADC for hardware trigger and DMA
  // ADTR: something about "initiating a conversion following the
  // assertion of ADHWT input after a pulse of the ADHWTSn input.
  // DMAEN: issue ADC DMA request when any conversion is completed.
  // on page 764, there is also some discussion of having ADC and DAC
  // operating on the same PDB, but they talk about DACINT stuff.
  // I think that's when you're not using DMA, and instead using
  // interrupts.  DAC below is triggered by a DMA_MUX_PDB signal.
  // adc check?
  ADC0_SC2 |= ADC_SC2_ADTRG | ADC_SC2_DMAEN;
    
  // source of the data
  // this is the real address - 
  adc0_dma.TCD->SADDR = &ADC0_RA;
  // DBG I want to read the PDB counter.  But that did crazy things
  // and the results made no sense.
  // adc0_dma.TCD->SADDR = &PDB0_CNT;
  // I want the high 4 bits of DAC0_C2, but to make this a 16 bit
  // read, start one byte earlier.
  // Nope - this also created ADC sequence errors.
  // adc0_dma.TCD->SADDR = &DAC0_C1;
      
  // address stride per request - just reads same register
  adc0_dma.TCD->SOFF = 0;
  // src size, 1 means 16 bits, dest size, same, 16 bits
  adc0_dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
  // each request moves 2 bytes
  adc0_dma.TCD->NBYTES_MLNO = 2;
  // after a major loop, adjust the source address this much, i.e. none!
  adc0_dma.TCD->SLAST = 0;
  // where to store the data
  adc0_dma.TCD->DADDR = adc0_buffer;

  // advance the output location 2 bytes each time
  adc0_dma.TCD->DOFF = 2;
  // not using scatter gather.  this is the destination equivalent of
  // SLAST.
  adc0_dma.TCD->DLASTSGA = -(sizeof(adc0_buffer)/dac_out_stride);
    
  // major loop - current number
  adc0_dma.TCD->CITER_ELINKNO = sizeof(adc0_buffer) / 2 / dac_out_stride;
    
  // major loop - beginning number(?)
  adc0_dma.TCD->BITER_ELINKNO = sizeof(adc0_buffer) / 2 / dac_out_stride;
    
  adc0_dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;

  // Implicit here is that the PDB triggers the ADC, and the completion
  // of the ADC triggers the DMA transfer of the result.
  // Possible, but a lesser solution to trigger the transfer via
  // DMAMUX_SOURCE_PDB - would require managing the delay of the
  // ADC trigger to avoid conflicts between the conversion and copying.

  // This is what it's supposed to be (and how it's done in library code)
  adc0_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);

  adc0_dma.enable();
  adc0_dma.attachInterrupt(adc0_dma_isr);
}

void SeaDuck::adc1_setup(void) {
  // this is probably wrong, as I really want to be setting up the 12/13
  // differential here - not sure where CCSENSE1 will get set.
  
  // analogRead(CCSENSE2);  // this hangs if it goes to ADC0.  but CCSENSE2 should force ADC1.
  ADC1_SC1A = 0 + ADC_SC1_DIFF; // (1<<5); // set differential

  ADC1_SC2 |= ADC_SC2_ADTRG | ADC_SC2_DMAEN;
    
  adc1_dma.TCD->SADDR = &ADC1_RA;
  adc1_dma.TCD->SOFF = 0;
  adc1_dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
  adc1_dma.TCD->NBYTES_MLNO = 2;
  // after a major loop, adjust the source address this much, i.e. none!
  adc1_dma.TCD->SLAST = 0;
  // where to store the data
  adc1_dma.TCD->DADDR = adc1_buffer;
  // advance the output location 2 bytes each time
  adc1_dma.TCD->DOFF = 2;
  // not using scatter gather.  this is the destination equivalent of
  // SLAST.
  adc1_dma.TCD->DLASTSGA = -( sizeof(adc1_buffer)/ dac_out_stride ); 
  // major loop - current number
  adc1_dma.TCD->CITER_ELINKNO = sizeof(adc1_buffer) / 2 / dac_out_stride;
  // major loop - beginning number(?)
  adc1_dma.TCD->BITER_ELINKNO = sizeof(adc1_buffer) / 2 / dac_out_stride;
    
  adc1_dma.TCD->CSR = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;

  // Implicit here is that the PDB triggers the ADC, and the completion
  // of the ADC triggers the DMA transfer of the result.
  // Possible, but a lesser solution to trigger the transfer via
  // DMAMUX_SOURCE_PDB - would require managing the delay of the
  // ADC trigger to avoid conflicts between the conversion and copying.
  
  // This is what it's supposed to be (and how it's done in library code)
  adc1_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);

  adc1_dma.enable();
  adc1_dma.attachInterrupt(adc1_dma_isr);
}

void SeaDuck::dac_init(void) {
  // system clock gating control register - prerequisite for
  // enabling the DAC is to get the clock to it
  SIM_SCGC2 |= SIM_SCGC2_DAC0;

  // this gives a 3.3V reference, and enables interrupts/DMA
  // for watermark and top of buffer.
  // trigger is left as hardware
  //   DAC_C0_DACBWIEN | DAC_C0_DACBTIEN;  -- this was getting some corruption,
  //    - like it would loop around and output 3 stale samples before the interrupt
  //      would fill in new samples.
  //   DAC_C0_DACBWIEN | DAC_C0_DACBBIEN; -- this looked good for dac_oversample=1,
  //   but gets some corruption with dac_oversample=2
  //   DAC_C0_DACBTIEN | DAC_C0_DACBBIEN -- looked good for one time at dac_oversample=2
  //   but somehow that was a fluke, and since it has looked just as bad.

  DAC0_C0 = DAC_C0_DACEN | DAC_C0_DACRFS;
  // These are set in dac_setup:  DAC_C0_DACBTIEN | DAC_C0_DACBBIEN;

  // set it up with a nice midpoint level:
  *(uint16_t *)&(DAC0_DAT0L) = dac_mid;
}

void SeaDuck::dac_setup(void) {
  // system clock gating control register - prerequisite for
  // enabling the DAC is to get the clock to it
  SIM_SCGC2 |= SIM_SCGC2_DAC0;

  // repeats some of the dac_init() work
  
  // this gives a 3.3V reference, and enables interrupt/DMA for
  // top and watermark.
  DAC0_C0 = DAC_C0_DACEN | DAC_C0_DACRFS | DAC_C0_DACBWIEN | DAC_C0_DACBTIEN;

  // interrupts are instead DMA requests, DAC will use buffer,
  // watermark is triggered 4 samples before the end,
  // and work mode is circular buffer.
  DAC0_C1 = DAC_C1_DMAEN | DAC_C1_DACBFEN | DAC_C1_DACBFWM(3) | DAC_C1_DACBFMD(0);

  // initialize the read pointer to 0, and promise to use all 16 samples of the
  // buffer.  Note that BFUP stores the last valid, not the count.
  // Read pointer is good at 0.  1 is bad.
  DAC0_C2 = DAC_C2_DACBFRP(0) | DAC_C2_DACBFUP(15);

  // clear any flags which would otherwise trigger some annoying preloading.
  DAC0_SR=0;
  
  //--------------------------------------------------------------------------------

  // source of the DMA data
  dac_dma.TCD->SADDR = dac_buffer;

  // address stride per datum
  dac_dma.TCD->SOFF = 4;

  // transfer configuration 
  // DMOD: output address is modulo this number of bits
  // DAC buffer is 32 bytes, so we need 5 bits
  // SSIZE/DSIZE(1) for 16 bit copies
  // SSIZE/DSIZE(2) for 32 bit copies
  dac_dma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2)
    | (5<<3) /* DMA_DCR_DMOD(5) */ ;
  
  // new: for each request, 8 2 byte samples are transferred, but now
  // we're using 32 bit copies - same thing, 4x 4 byte samples.
  dac_dma.TCD->NBYTES_MLNO = 8*2; // or 4*4 bytes
  
  // increment after the major loop finishes.  so this should just send us back to
  // the beginning of the buffer, effectively getting a circular buffer.
  dac_dma.TCD->SLAST = -(sizeof(dac_buffer)/dac_out_stride);
  // where to put the data.
  // for the DMOD usage, DAT0L is all zeros for the last 12 bits, no problems
  // there.
  dac_dma.TCD->DADDR = &DAC0_DAT0L;
  // old code: after each write, we don't shift the output address
  // dac_dma.TCD->DOFF = 0;
  dac_dma.TCD->DOFF = 4; // each write advances 2 bytes, but now 4 bytes with 32-bits.
  
  // current major loop number - counts down.
  // new code - reduce by the size of the minor loop (2*8), and the stride, too.
  dac_dma.TCD->CITER_ELINKNO = sizeof(dac_buffer) / (2*8) / dac_out_stride;
  // SGA naming is for a feature we're not using.  This is the destination
  // equivalent of SLAST.  with the DMOD stuff, we don't need this.
  dac_dma.TCD->DLASTSGA = 0;
  
  // must start off same as above CITER.  it's the reset
  // value for the major loop count.
  // see CITER note for /8 here.
  dac_dma.TCD->BITER_ELINKNO = sizeof(dac_buffer) / (2*8) / dac_out_stride;

  // not using interrupts here
  dac_dma.TCD->CSR = 0; // DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;

  // This ties it to DAC buffer interrupts/DMAs.
  dac_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_DAC0);  

  dac_dma.enable();

  // Trigger it twice by hand to load the first 16 samples
  dac_dma.triggerManual();
  // it only needs to copy 4 4byte samples, ought to take less than
  // a microsecond.  
  delay(1); 
  dac_dma.triggerManual();

}

// Just for debugging
void SeaDuck::dac_status(void) {
  int read_ptr=DAC0_C2>>4;
  // int dac_idx=((uint16_t*)&DAC0_DAT0L)[read_ptr];
  int word_soffset=(uint16_t*)dac_dma.TCD->SADDR - dac_buffer;
  int word_doffset=(uint16_t*)dac_dma.TCD->DADDR - (uint16_t*)&DAC0_DAT0L;
  // int adc_offset=(uint16_t*)adc0_dma.TCD->DADDR - adc0_buffer;

  int loop_count=adc0_n;
  
  // Serial.print("# DAC output val            =");
  // Serial.println( dac_idx );
  
  Serial.print("# DAC read pointer          =");
  Serial.println( read_ptr ); 
  
  // Serial.print("# ADC0 DMA DADDR word offset =");
  // Serial.println( adc_offset);

  Serial.print("# DAC DMA SADDR word offset =");
  Serial.println( word_soffset);

  Serial.print("# DAC DMA DADDR word offset =");
  Serial.println( word_doffset);

  Serial.print("# DAC SR =" );
  Serial.println( DAC0_SR );
  
  // These don't really make any sense if the dac buffer isn't holding
  // a ramp signal.
  // Serial.print("# DAC index erro    =");
  // Serial.print( (256 + dac_per_adc*adc_offset
  //                -
  //                dac_idx) % 256 );
  Serial.print("#    adc loop=");
  Serial.println(loop_count);
  
  // for(int i=0;i<16;i++) {
  //   Serial.print("# "); Serial.print(i) ; Serial.print(" => ");
  //   Serial.print( ((uint16_t *)(&DAC0_DAT0L))[i] );
  //   // And show the dac_buffer
  //   // Serial.print( "   dac_buffer: ");
  //   // Serial.print( dac_buffer[i] );
  //   if ( read_ptr == i )
  //     Serial.print(" << ");
  //   Serial.println();
  // }
}

void SeaDuck::pdb_setup(void) {
  // set the programmable delay block to trigger DMA requests
  // audio library tests before setting - but no need here to worry
  // about prior state
  SIM_SCGC6 |= SIM_SCGC6_PDB; // enable clock to programmable delay block
  
  PDB0_MOD = pdb_period; // period of the counter, 16 bits
  
  // channel control register 1, for channel 0
  // sets TOS and EN both to 1.  This controls firing of the ADC by the 
  // PDB.
  // TOS: trigger output select.  EN: enable.
  // BB: back-to-back, not enabled.
  PDB0_CH0C1 = 0x0101;
  // trigger ADC1, too??
  PDB0_CH1C1 = 0x0101;

  // could be up to PDB_MOD - this controls when during the PDB count-up
  // ADC0 fires.  Not sure what the optimal setting is here.  the DAC is updated
  // at 1.  PDB_MOD is 1087, but that will change.
  // For the moment, trigger ADC halfway through.  At low speed (50kHz) this
  // is plenty close, and we get a 0.0 sample phase lag.

  // this has been working very well:
  // PDB0_CH0DLY0 = pdb_period/2;
  // PDB0_CH1DLY0 = pdb_period/2;
  // having them the same, but smaller, make a small difference but basicaly
  // the same behavior.
  // having ch1 a little bit later flipped the typical sign of the spikes,
  // having ch1 significantly later...  likewise... blech.
  // having ch0 significant later -- looks even worse
  // having both much closer to 0?  Doesn't seem to help at all.
  PDB0_CH0DLY0 = 2; // does this help at all? not really.
  PDB0_CH1DLY0 = 2;
  
  // New setup for DAC
  PDB0_DACINTC0 = 0x01; // no external trigger, interval trigger enabled
  PDB0_DACINT0 = (pdb_period / dac_per_adc); // had tried a -1 for DBG
  // Serial.print("# pdb period: "); Serial.println(pdb_period);
  // Serial.print("# dac interval: "); Serial.println( pdb_period / dac_per_adc );
  
  // new location of ldok
  // is it okay to move these after the delay settings?
  PDB0_SC = LOC_PDB_CONFIG | PDB_SC_LDOK | PDB_SC_PRESCALER(pdb_prescaler);
  // manually trigger the loop:
  PDB0_SC = LOC_PDB_CONFIG | PDB_SC_PRESCALER(pdb_prescaler) | PDB_SC_SWTRIG;
}

void SeaDuck::pdb_stop(void) {
  // old way, sort of abrupt:
  // PDB0_SC &= ~PDB_SC_PDBEN;
  // new way - try to round out the loop:
  
  // PDB0_SC = LOC_PDB_CONFIG | PDB_SC_LDOK | PDB_SC_PRESCALER(pdb_prescaler);

  // change load mode to be at the end of the loop,
  // drop continuous mode and enable.
  PDB0_SC = (PDB_SC_TRGSEL(15) |  \
             PDB_SC_LDOK | PDB_SC_LDMOD(1) );
  while ( PDB0_SC & PDB_SC_LDOK ) ; // spin while it finishes loop.
  // that didn't help.
}

void SeaDuck::adc0_stop(void) {
  // undo adc_setup()

  // this keeps the reference selection, but clears everything else.
  // in particular, hardware triggering seems send analogRead() into
  // an infinite loop.
  ADC0_SC2 = ADC0_SC2 & 0x03; // undoing ADC_SC2_ADTRG | ADC_SC2_DMAEN;

  adc0_dma.disable();
  adc0_dma.detachInterrupt();
}

void SeaDuck::adc1_stop(void) {
  ADC1_SC2 = ADC1_SC2 & 0x03; // undoing ADC_SC2_ADTRG | ADC_SC2_DMAEN;

  adc1_dma.disable();
  adc1_dma.detachInterrupt();
}

void SeaDuck::adc_stop(void) {
  adc0_stop();
  adc1_stop();
}

void SeaDuck::dac_stop() {
  dac_dma.disable();
  *(uint16_t *)&(DAC0_DAT0L) = dac_mid;
}
 
void SeaDuck::setup() {
  Shell::setup();

  dac_init();
  adc_init();
  // enable clock to programmable delay block
  // so that dac_status() can read PDB registers whenever.
  SIM_SCGC6 |= SIM_SCGC6_PDB; 
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
  }
  else {
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

void SeaDuck::scan_setup() {
  Serial.println("# About to adc setup");
  fill_sine_buffer();

  // DBG - hasn't done anything.  commenting, but leaving for a little while.
  // ERCA is *not* set, so it should be using priorities.
  // DMA_CR |= DMA_CR_HOE;

  // this gets the counts stable +-1
  // DMAPriorityOrder(adc0_dma, adc1_dma; 
  
  adc_setup();
  dac_setup();

  pdb_setup();  

  Serial.println("# Setup complete");
}

void SeaDuck::scan_cleanup() {
  pdb_stop();
  adc_stop();
  dac_stop();
}

void SeaDuck::scan() {
  scan_setup();

  scan_loop();

  scan_cleanup();
}

void SeaDuck::scan_loop() {
  int j;
  
  Serial.print("#");
  while( (adc0_n < adc_n) ||
         (adc1_n < adc_n) ) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("done");
  
  Serial.println("idx,dac,adc0_mean,adc0_var,adc1_mean,adc1_var");
  float mean, variance;
  
  for(j=0;j<buff_n_samples;j++) {
    // not sure why it's losing so much. this didn't help:
    // while( !Serial.availableForWrite() ) ; // dangerous???
    // delay(5); // 50 fixed it, but was painful.
    delay(2); // annoying, but still having some corruption.
    
    Serial.print(j); Serial.print(",");
    Serial.print(dac_buffer[j]); Serial.print(",");
    
    mean = adc0_accum[j] / (float)(adc0_n-adc_n_discard);
    variance = (float)adc0_variance[j] * (1<<adc_var_shift) / (float)(adc0_n-adc_n_discard) - (mean*mean);
    Serial.print(mean); Serial.print(",");
    Serial.print(variance); Serial.print(",");
    
    mean = adc1_accum[j] / (float)(adc1_n-adc_n_discard);
    variance = (float)adc1_variance[j] * (1<<adc_var_shift) / (float)(adc1_n-adc_n_discard) - (mean*mean);
    Serial.print(mean); Serial.print(",");
    Serial.print(variance); 
    
    Serial.println();
  }
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
