/*
 * Proof of concept using samd21 to read from an 
 * inductive conductivity sensor
 * low-level details largely based on https://github.com/manitou48/ZERO/blob/master/adcdma.ino
 * and dacdma.ino
 */

// http://www.atmel.com/Images/Atmel-42258-ASF-Manual-SAM-D21_AP-Note_AT07627.pdf pg 73

#define ADCPIN A1
#define ADC_HWORDS 32
uint16_t adcbuf[ADC_HWORDS];     

#define DAC_HWORDS 16
uint16_t dacbuf[DAC_HWORDS];

typedef struct {
    uint16_t btctrl;
    uint16_t btcnt;
    uint32_t srcaddr;
    uint32_t dstaddr;
    uint32_t descaddr;
} dmacdescriptor ;

volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));
volatile dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor __attribute__ ((aligned (16)));

static uint32_t chnl = 0;  // DMA channel
volatile uint32_t dmadone;
volatile uint8_t active_channel=0;

// From the DAC sketch:
static uint32_t dacdma_chnl = 1;  // DMA channel for DAC

void memcpy_vol(volatile void *dest, void *src, int nbytes) {
  int i;
  for(i=0;i<nbytes;i++) {
    ((char *)dest)[i]=( (char*)src )[i];
  }
}


void DMAC_Handler() {
  // interrupts DMAC_CHINTENCLR_TERR DMAC_CHINTENCLR_TCMPL DMAC_CHINTENCLR_SUSP
  // uint8_t active_channel;

  // disable irqs ?
  __disable_irq();
  active_channel =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // get channel number
  // if (active_channel==chnl) {
  DMAC->CHID.reg = DMAC_CHID_ID(active_channel);
  dmadone = DMAC->CHINTFLAG.reg;
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_SUSP;
  __enable_irq();
  if( ! dmadone ) {
    Serial.println("DMAC_Handler -- did not set dmadone");
  }
}

// ADC version
void dma_init() {
  // probably on by default
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC ;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC ;
  NVIC_EnableIRQ( DMAC_IRQn ) ;

  // Global DMA settings to tell it where we store all dmac descriptors
  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
  DMAC->WRBADDR.reg = (uint32_t)wrb;
  // above have to be set for DMAENABLE.
  // Enable DMAC, and enable I think all priority levels.
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
}

// DAC version
void dac_dma_init() {
  // increment the DAC output per ADC sample.
    uint32_t temp_CHCTRLB_reg;

    // This starts to be DAC-specific I think
    // Tell the  DMAC we're going to configure the DAC DMA channel
    DMAC->CHID.reg = DMAC_CHID_ID(dacdma_chnl); // RH changed this dac-specific channel.
    // Make sure it's disabled.
    DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
    // I think this just resets the channel to a known state.
    DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
    // I think this means we will *not* get a software trigger generated
    DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << dacdma_chnl));

    // 
    temp_CHCTRLB_reg = DMAC_CHCTRLB_LVL(0) | // different priority than ADC.  meh?
      DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY) | // instead of the TCC overflow stuff.
      DMAC_CHCTRLB_TRIGACT_BEAT; // one trigger required per beat
    DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
    descriptor.descaddr = (uint32_t) &descriptor_section[dacdma_chnl];   // circular
    descriptor.srcaddr = (uint32_t)dacbuf + DAC_HWORDS*2;
    descriptor.dstaddr = (uint32_t)&DAC->DATA.reg;
    descriptor.btcnt =  DAC_HWORDS;
    // defaults to stepsize=0x0 = stride 1
    // DST does not increment. each beat is 16 bits, a half-word.
    // this descriptor is valid.
    descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID;
    memcpy_vol(&descriptor_section[dacdma_chnl],&descriptor, sizeof(dmacdescriptor));

    // start channel
    DMAC->CHID.reg = DMAC_CHID_ID(dacdma_chnl);
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

// log ADC_HWORDS ADC samples
void adc_dma(void *rxdata,  size_t hwords) {
  uint32_t temp_CHCTRLB_reg;
  
  DMAC->CHID.reg = DMAC_CHID_ID(chnl);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << chnl));

  temp_CHCTRLB_reg = DMAC_CHCTRLB_LVL(1) |
    DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY) | DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK ; // enable all 3 interrupts

  dmadone = 0;
  descriptor.descaddr = 0; // not circular, so one and done.
  descriptor.srcaddr = (uint32_t) &ADC->RESULT.reg;
  descriptor.btcnt =  hwords;
  // end address! docs suggest this is the "last" valid address.
  // so it should be hwords-1?  no - the last element never gets filled
  // in then.
  descriptor.dstaddr = (uint32_t)rxdata + hwords*2;
  descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID;
  memcpy_vol(&descriptor_section[chnl],&descriptor, sizeof(dmacdescriptor));

  // start channel
  DMAC->CHID.reg = DMAC_CHID_ID(chnl);
  // an ADCsync() here did not help
  ADCsync(); 
  ADC->SWTRIG.reg |= 0x01; // FLUSH
  ADCsync();
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void adc_dma_status(void) {
  DMAC->CHID.reg = DMAC_CHID_ID(active_channel);
  Serial.print("CHINTFLAG: "); Serial.println(DMAC->CHINTFLAG.reg);
  Serial.print("CHCTRLA:   "); Serial.println(DMAC->CHCTRLA.reg,HEX);
  Serial.print("CHCTRLB:   "); Serial.println(DMAC->CHCTRLB.reg,HEX);
  Serial.print("CHSTATUS:   "); Serial.println(DMAC->CHSTATUS.reg,HEX);
  Serial.print("BUSYCH: "); Serial.println(DMAC->BUSYCH.reg);
  Serial.print("INTSTATUS: "); Serial.println(DMAC->INTSTATUS.reg);

  Serial.print("ADC STATUS: "); Serial.println(ADC->STATUS.reg,HEX);
  Serial.print("ADC INTFLAG: "); Serial.println(ADC->INTFLAG.reg,HEX);
  Serial.print("ADC CTRLA: "); Serial.println(ADC->CTRLA.reg,HEX);
  
  Serial.print("ADC CTRLB: "); Serial.println(ADC->CTRLB.reg,HEX);
  Serial.print("ADC SWTRIG: "); Serial.println(ADC->SWTRIG.reg,HEX);
  Serial.print("ADC EVCTRL: "); Serial.println(ADC->EVCTRL.reg,HEX);
  
  Serial.print("WRB[adc dma]  btctrl   "); Serial.println(wrb[chnl].btctrl);
  Serial.print("              btcnt    "); Serial.println(wrb[chnl].btcnt);
  Serial.print("              srcaddr  "); Serial.println(wrb[chnl].srcaddr);
  Serial.print("              dstaddr  "); Serial.println(wrb[chnl].dstaddr);
  Serial.print("              descaddr "); Serial.println(wrb[chnl].descaddr);
}

void adc_init(){
  analogRead(ADCPIN);  // do some pin init pinPeripheral()
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  ADCsync();
  //ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; //  2.2297 V Supply VDDANA
  //ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain select as 1X
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;  // default
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;
  ADCsync();    //  ref 31.6.16
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ADCPIN].ulADCChannelNumber;
  ADCsync();
  ADC->AVGCTRL.reg = 0x00 ;       //no averaging
  ADC->SAMPCTRL.reg = 0x00;  ; //sample length in 1/2 CLK_ADC cycles
  ADCsync();
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | ADC_CTRLB_FREERUN | ADC_CTRLB_RESSEL_10BIT;
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;
  ADCsync();
}



void setup(){
  int i;
  float phase = 3.14159 * 2./DAC_HWORDS;

  Serial.begin(115200);
  Serial.println("Inductive Conductivity Testing");

  for (i=0;i<DAC_HWORDS;i++) dacbuf[i]= sinf(i*phase) * 500.0f + 512.0f;
  analogWriteResolution(10);
  analogWrite(A0,123);   // DAC init setup DAC pin and zero it

  // ADC 
  adc_init();
  dma_init();

  // DAC code
  dac_dma_init();   // will increment DAC output per ADC sample.
}

void loop() {
  uint32_t t,i;
  double freq;
  Serial.println("Top of loop");

  for(i=0;i<ADC_HWORDS;i++) {
    adcbuf[i]=0;
  }

  t = micros();
  adc_dma(adcbuf,ADC_HWORDS);

  adc_dma_status();
  
  Serial.println("Waiting");
  while( !dmadone ) { // await DMA done isr
    if ( micros() - t > 5000 ) {
      Serial.println("Failed to get out of dmadone wait");
      adc_dma_status();

      // diagnose where it stopped?
      for(i=0;i<ADC_HWORDS;i++) {
        Serial.println(adcbuf[i]);
      }

      // And print a few updated conversions
      for (i=0;i<5;i++){
        Serial.print("ADC result:");
        ADCsync();
        Serial.println(ADC->RESULT.reg);
        delay(20);
      }
      while(1); // stop so that output remains visible.
    }
  }
  t = micros() - t;

  freq = 1e3 * float(ADC_HWORDS) / t;

  Serial.print("dmadone: "); Serial.println(dmadone);
  Serial.print("int channel: "); Serial.println(active_channel);
  Serial.print("----");
  Serial.print(t);
  Serial.print(" us  ");
  Serial.print(freq);
  Serial.println(" kHz ----");
  
  for(i=0;i<ADC_HWORDS;i++) {
    Serial.print(dacbuf[i%DAC_HWORDS]);
    Serial.print("   ");
    Serial.println(adcbuf[i]);
  }
  Serial.println("----");

  delay(500);

  // And print a few updated conversions
  for (i=0;i<5;i++){
    Serial.print("ADC result:");
    ADCsync();
    Serial.println(ADC->RESULT.reg);
    delay(20);
  }

  Serial.println("End of loop");
}

// This prints typ. 2055 us   425
// ADC_HWORDS is 1024
// Not sure why dstaddr gets set to the end address.

// That's working sometimes.
// but then sometimes it's just getting alternating 0, 1023 on the ADC.
// and eventually hangs??

// Disabling the DAC code seems to make the ADC code more stable.
// hmm.. This time it got through 6 reads, then froze.
// try switching priorities?
// DMAC_CHCTRLB_LVL(0) for DAC, and 1 for ADC?
// No - eventually freezes this way, too.

// Can I find where it's freezing?
// With the added print statements it has been running for a lot longer
// now. but it eventually froze, and just after the 'top of loop' message.

// 
// Not sure why there is a lag at the start of the ADC readings.
// Shouldn't this be running full-steam all the time?
//   unless the ADC doesn't run free when nobody is reading its output.
//   in that case the ADC and DAC are sitting still until DMA starts.
//   in that case, a failure to trigger an initial DMA would also mean
//   that the ADC would never start firing.

// TODO:
// Read through the App note to see if there are additional considerations
// for getting the ADC and DAC to play nicely together.
//    No help.  They are running a single DMA transfer, straight from ADC to
//    DAC.
// Have to go to the datasheet.
//  -- advice on priorities?
//  -- any way to chain the DMA transfers, rather than having them trigger
//     on the same event?  I don't think so.
//  -- dissect adc_dma -- added several print statements...

// consider having interrupt handler only respond on adc dma channel
//  -- if it freezes


// Try to figure out why there is a lag in the ADC values.
// the ADC buffer is a multiple of the DAC buffer size, so once
// the ADC is in freerun mode, it should remain in phase with the
// DAC output.
//  Can also print dmadone, to see if we're occasionally getting
//  DMA errors.
// Eventually froze again -- after the end of adc_dma::E, but
// I guess dmadone never got set?

// with new output:
// int channel is always coming out 0.  Good.
// dmadone==2.  That's TCMPL.  Good.

// it eventually timed out waiting for dmadone.  no indication of why.
// previous interrupts look exactly the same.

// Next thing is to check on status flags, before and after
// loop times out

// Typical output now:
// CHINTFLAG: 0
// CHCTRLA: 0  // rarely 2
// CHCTRLB: 802720
// CHSTATUS: 0

// When it does freeze, CHCTRLA is 2 both at the end of adc_dma
// and when it fails out.  That corresponds to the ENABLE bit --
// so I think it just means that the channel is still enabled?
// not sure how it would not always be 2 at the end adc_dma?

// No luck.
// Is it possible that the ADC is having some issue?

// typical values:
// ADC STATUS: 0
//     INTFLAG: 0x0B == SYNCRDY | OVERRUN | RESRDY ?
//     CTRLA: 2 == ENABLE ( no run-in-standy, no software reset underway)
//     CTRLB: 0x0224 == prescaler 0x02=div16 | RES 10 bit | FREERUN
// no different when it fails.

// when it fails, only the first entry has been written to.
// print a few more ADC results (have to read the register directly I think)
//    -- is the ADC still in freerun, and just the DMA stalled out?
//    -- or did the ADC error out and stop
// Looks like each read triggers a DAC update, even if spaced out.
// when it fails, I get a single value read like 510.
// then I read the register a few more times and get 509, 501 699, 860, 969.
// even though I'm pausing 20ms between each of those.

// does this occur if I manually read some more afterwards?
// seems to just pick up two samples of DAC output after the last seen
// in the buffer..
// like the adcbuf ends with dac[5], we miss 6,7 and a late read gets
// dac[8], dac[9], ...


//  will need to spin on SYNCBUSY, probably
//  print ADC->SWTRIG, ADC->EVCTRL.

//  the result register is read-synchronized. Is it possible that the timing is
//  off here?

// Is anything not volatile that should be volatile?  MAYBE - the descriptors.
// Hack at that. not sure that I got the volatile stuff done correctly.
// still freezes.

// Can print BTCNT in the writeback descriptor
// typically btctrl 0x2304
//   btcnt 0
//   sraddres 1107312666
//   dstaddre 536871294
//   descadress 0

// SWTRIG=0  EVCTRL=0  -- unchanging

// When it freezes
//    btctrl 2305 -- extra bit means the descriptor is still VALID.
//    btcnt 32 -- means that it hasn't incremented at all?
// others the same.


// Could print DMAC->ACTIVE.reg

// I'm wondering if it is going *too* fast.  Without the print statements,
// I was seeing over 400kSPS, while it is spec'd at 350kSPS.
// would it be safer if I didn't push it like that?


// Is there some part of the DMA process that should wait on the ADC?
// try just before enabling the channel.
// it went into the 0,1023,0,1023 bit for a while, and finally froze.
// no difference in the register values.


// What if I flush the ADC before starting DMA?  So far so good....????
//  still going.

