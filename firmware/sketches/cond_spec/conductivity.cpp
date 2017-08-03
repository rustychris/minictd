extern ADC *adc;

#include "conductivity.h"

// Sequencing of ADC usage (for now, analog power bus is always on)
// 

// since ADC and Audio define this differently, define our own here.
// the only real difference is DMAEN.  That means that PDB triggers
// a dma transfer on looping, rather than an interrupt.
// TRGSEL(15): software trigger
// dropped PDBIE and DMAEN as they are no longer needed for the DAC.
#define LOC_PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_CONT  \
                        | PDB_SC_LDMOD(0) )

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

Conductivity::Conductivity() {
  // set some basic, sane settings
  pdb_period=1025;
  dac_per_adc=2;
  adc_var_shift=4;
  adc_n=20;
}

void Conductivity::init(void) {
  // enable clock to programmable delay block
  // so that dac_status() can read PDB registers whenever.
  SIM_SCGC6 |= SIM_SCGC6_PDB;

  // -- dac_init --

  // system clock gating control register - prerequisite for
  // enabling the DAC is to get the clock to it
  SIM_SCGC2 |= SIM_SCGC2_DAC0;

  // this gives a 3.3V reference, and enables interrupts/DMA
  // for watermark and top of buffer.
  // trigger is left as hardware
  DAC0_C0 = DAC_C0_DACEN | DAC_C0_DACRFS;
  // These are set in dac_setup:  DAC_C0_DACBTIEN | DAC_C0_DACBBIEN;

  // set it up with a nice midpoint level:
  // note that this should only be done when the analog supply is on!
  *(uint16_t *)&(DAC0_DAT0L) = dac_mid;
}

void Conductivity::fill_sine_buffer() {
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

void Conductivity::adc_setup(void) {
  adc0_setup();
  adc1_setup();
}

void Conductivity::adc0_setup(void) {
  // Configure the ADC and run at least one software-triggered
  // conversion.  This completes the self calibration stuff and
  // leaves the ADC in a state that's mostly ready to use.
  // to be clear - writes to SC1A initiate a conversion
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

  // ---- The rest is DMA ----
  
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

void Conductivity::adc1_setup(void) {
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

void Conductivity::dac_setup(void) {
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
void Conductivity::dac_status(void) {
  int read_ptr=DAC0_C2>>4;
  int word_soffset=(uint16_t*)dac_dma.TCD->SADDR - dac_buffer;
  int word_doffset=(uint16_t*)dac_dma.TCD->DADDR - (uint16_t*)&DAC0_DAT0L;

  int loop_count=adc0_n;
  
  Serial.print("# DAC read pointer          =");
  Serial.println( read_ptr ); 
  
  Serial.print("# DAC DMA SADDR word offset =");
  Serial.println( word_soffset);

  Serial.print("# DAC DMA DADDR word offset =");
  Serial.println( word_doffset);

  Serial.print("# DAC SR =" );
  Serial.println( DAC0_SR );
  
  Serial.print("#    adc loop=");
  Serial.println(loop_count);
}

void Conductivity::pdb_setup(void) {
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

  PDB0_CH0DLY0 = 2; // does this help at all? not really.
  PDB0_CH1DLY0 = 2;
  
  // New setup for DAC
  PDB0_DACINTC0 = 0x01; // no external trigger, interval trigger enabled
  PDB0_DACINT0 = (pdb_period / dac_per_adc); 
  
  // new location of ldok
  // is it okay to move these after the delay settings?
  PDB0_SC = LOC_PDB_CONFIG | PDB_SC_LDOK | PDB_SC_PRESCALER(pdb_prescaler);
  // manually trigger the loop:
  PDB0_SC = LOC_PDB_CONFIG | PDB_SC_PRESCALER(pdb_prescaler) | PDB_SC_SWTRIG;
}

void Conductivity::pdb_stop(void) {
  // round out the loop:
  // change load mode to be at the end of the loop,
  // drop continuous mode and enable.
  PDB0_SC = (PDB_SC_TRGSEL(15) |  \
             PDB_SC_LDOK | PDB_SC_LDMOD(1) );
  while ( PDB0_SC & PDB_SC_LDOK ) ; // spin while it finishes loop.
}

void Conductivity::adc_stop(void) {
  // undo adc_setup()

  // this keeps the reference selection, but clears everything else.
  // in particular, hardware triggering seems send analogRead() into
  // an infinite loop.

  // ADC0
  ADC0_SC2 = ADC0_SC2 & 0x03; // undoing ADC_SC2_ADTRG | ADC_SC2_DMAEN;
  adc0_dma.disable();
  adc0_dma.detachInterrupt();

  // ADC1  
  ADC1_SC2 = ADC1_SC2 & 0x03; // undoing ADC_SC2_ADTRG | ADC_SC2_DMAEN;
  adc1_dma.disable();
  adc1_dma.detachInterrupt();
}

void Conductivity::dac_stop() {
  dac_dma.disable();
  *(uint16_t *)&(DAC0_DAT0L) = dac_mid;
}

// Higher level steps:
void Conductivity::scan_setup() {
  Serial.println("# About to adc setup");
  fill_sine_buffer();

  adc_setup();
  dac_setup();
  pdb_setup();  

  Serial.println("# Setup complete");
}

void Conductivity::scan_cleanup() {
  pdb_stop();
  adc_stop();
  dac_stop();
}

void Conductivity::scan_loop() {
  int j;
  
  Serial.print("#");
  while( (adc0_n < adc_n) ||
         (adc1_n < adc_n) ) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("done");
}

void Conductivity::scan_dump() {
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

void Conductivity::scan() {
  scan_setup();

  scan_loop();

  scan_dump();
  
  scan_cleanup();
}

void Conductivity::read() {
  scan_setup();

  scan_loop();

  scan_reduce();
  
  scan_cleanup();
}

void Conductivity::scan_reduce() {
  // HERE
  reading=3.14159; // BOGUS
}
