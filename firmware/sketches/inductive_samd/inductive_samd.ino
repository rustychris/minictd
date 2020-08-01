/*
 * Proof of concept using samd21 to read from an 
 * inductive conductivity sensor
 * low-level details largely based on https://github.com/manitou48/ZERO/blob/master/adcdma.ino
 * and dacdma.ino
 */

// http://www.atmel.com/Images/Atmel-42258-ASF-Manual-SAM-D21_AP-Note_AT07627.pdf pg 73

#include <arduinoFFT.h>

#define ADCPIN A1
// This has to be a power of 2
#define NSAMPLES_FFT 1024
// as does this
#define DAC_HWORDS 32
#define ADC_HWORDS (NSAMPLES_FFT+DAC_HWORDS)

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

uint16_t adcbuf[ADC_HWORDS];     

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

static uint32_t adcdma_chnl = 0;  // DMA channel
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
  
  DMAC->CHID.reg = DMAC_CHID_ID(adcdma_chnl);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << adcdma_chnl));

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
  memcpy_vol(&descriptor_section[adcdma_chnl],&descriptor, sizeof(dmacdescriptor));

  // start channel
  DMAC->CHID.reg = DMAC_CHID_ID(adcdma_chnl);
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
  
  Serial.print("WRB[adc dma]  btctrl   "); Serial.println(wrb[adcdma_chnl].btctrl);
  Serial.print("              btcnt    "); Serial.println(wrb[adcdma_chnl].btcnt);
  Serial.print("              srcaddr  "); Serial.println(wrb[adcdma_chnl].srcaddr);
  Serial.print("              dstaddr  "); Serial.println(wrb[adcdma_chnl].dstaddr);
  Serial.print("              descaddr "); Serial.println(wrb[adcdma_chnl].descaddr);
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
  // ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | ADC_CTRLB_FREERUN | ADC_CTRLB_RESSEL_10BIT;
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 | ADC_CTRLB_FREERUN | ADC_CTRLB_RESSEL_12BIT;
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;
  ADCsync();
}



void setup(){
  int i;
  float phase = 3.14159 * 2./DAC_HWORDS;

  Serial.begin(115200);
  while(!Serial);
  Serial.println("Inductive Conductivity Testing");

  for (i=0;i<DAC_HWORDS;i++) dacbuf[i]= sinf(i*phase) * 100.0f + 512.0f;
  analogWriteResolution(10);
  analogWrite(A0,512);   // DAC init setup DAC pin and zero it

  // ADC 
  adc_init();
  dma_init();

  // DAC code
  dac_dma_init();   // will increment DAC output per ADC sample.
}

 
 
// void show(const char * s, cplx buf[]) {
// 	printf("%s", s);
// 	for (int i = 0; i < 8; i++)
// 		if (!cimag(buf[i]))
// 			printf("%g ", creal(buf[i]));
// 		else
// 			printf("(%g, %g) ", creal(buf[i]), cimag(buf[i]));
// }
 
 
void samples_postprocess(uint16_t *adcbuf, double sample_rate) {
  // FFT on adcbuf
  // report fundamental, harmonic distortion(?), and noise
  
  // Populate double arrays
  double phase_fund;
  double vReal[NSAMPLES_FFT];
  double vImag[NSAMPLES_FFT];
  double power;
  double pow_fund,pow_harm=0.0,pow_noise=0.0;
  int n_harm=0,n_noise=0;
  
  for(int i=0;i<NSAMPLES_FFT;i++) {
    // Take the latter part of the buffer
    vReal[i]=adcbuf[i+ADC_HWORDS-NSAMPLES_FFT];
    vImag[i]=0.0;
  }

  // No windowing since we're sampling an integral number
  // of periods
  FFT.Compute(vReal, vImag, NSAMPLES_FFT, FFT_FORWARD);
  phase_fund=atan2(vImag[NSAMPLES_FFT/DAC_HWORDS],
                   vReal[NSAMPLES_FFT/DAC_HWORDS]);
  FFT.ComplexToMagnitude(vReal, vImag, NSAMPLES_FFT);

  // Print the results
  for (int i = 0; i < NSAMPLES_FFT/2; i++)
    {
      double abscissa;
      /* Print abscissa value */
      abscissa = ((i * 1.0 * sample_rate) / NSAMPLES_FFT);
      
      if(0) {
        Serial.print(abscissa, 1);
        Serial.print("Hz");
        Serial.print(" ");
        Serial.println(vReal[i], 2);
      }
      
      power=vReal[i]/NSAMPLES_FFT;
      power=power*power;
      
      if(i>0) {
        if( i == NSAMPLES_FFT/DAC_HWORDS ) {
          pow_fund=power;
        } else if ( i % (NSAMPLES_FFT/DAC_HWORDS) == 0 ) {
          pow_harm += power;
          n_harm++;
        } else {
          pow_noise += power;
          n_noise++;
        }
      }
    }
  // Serial.println();
  Serial.print("Pow fundamental: "); Serial.print(pow_fund,3);
  Serial.print("   phase(deg): "); Serial.print(phase_fund*180/3.14159,2);
  Serial.print("   harmonic: "); Serial.print(pow_harm,3);
  Serial.print(" / "); Serial.print(n_harm);
  Serial.print(" = "); Serial.print(pow_harm/n_harm,3);
  Serial.print("   noise: "); Serial.print(pow_noise,3);
  Serial.print(" / "); Serial.print(n_noise);
  Serial.print(" = "); Serial.print(pow_noise/n_noise,3);
  Serial.println();
}

void loop() {
  uint32_t t,i;
  double freq,sig_freq;
  //Serial.println("Top of loop");

  for(i=0;i<ADC_HWORDS;i++) {
    adcbuf[i]=0;
  }

  t = micros();
  adc_dma(adcbuf,ADC_HWORDS);

  // adc_dma_status();
  // Serial.println("Waiting");
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
  sig_freq=freq/DAC_HWORDS;

  //  Serial.print("dmadone: "); Serial.println(dmadone);
  // Serial.print("int channel: "); Serial.println(active_channel);
  // Serial.print("----");
  // Serial.print(t);
  // Serial.print(" us  ");
  // Serial.print(freq);
  // Serial.print(" kHz");
  // Serial.print("  Signal ");
  // Serial.print(sig_freq);
  // Serial.println(" kHz ----");
  
  // for(i=0;i<DAC_HWORDS;i++) {
  //   Serial.print(dacbuf[i%DAC_HWORDS]);
  //   Serial.print("   ");
  //   Serial.println(adcbuf[i]);
  // }
  // Serial.println("----");

  samples_postprocess(adcbuf,freq*1000);

  delay(50);
  // while(1); 

  // // And print a few updated conversions
  // for (i=0;i<5;i++){
  //   Serial.print("ADC result:");
  //   ADCsync();
  //   Serial.println(ADC->RESULT.reg);
  //   delay(20);
  // }

  // Serial.println("End of loop");
}

// What if I flush the ADC before starting DMA?  So far so good....????
//  still going.
// Deleted a ton of notes on debugging the ADC freezing.

// Reading 12-bit drops the rate from 440kHz or so to 370 kHz.
// Writing 12-bit to the DAC didn't go so well. seems to often just
// spit out 4095 / 0.  but sometimes it gets real numbers.
// still very occasionally gets a glitch with 10-bit DAC and 12-bit ADC,
// namely the first entry being 0.


// With DAC amplitude of 1000 p-p, seems to saturate output when a short
// is in the toroid.
// 200p-p still saturates.
// 20p-p still saturates.
// 4 counts p-p? now I get a swing of about 1500, but it's not symmetric?
// Adjust code to extract the synchronous signal, phase and amplitude.
// then try this again with some test solutions.
// At that point consider a combination of a divider on the DAC output
// and/or a smaller transimpedance gain.
// Also print out the drive frequency in addition to the sample frequency.

// That's working
// I'm getting power of 39802 at the signal frequency of 13.27kHz
// 0.8 in harmonics
// 1.7 in noise.

// 42dB SINAD across the full bandwidth.
// So if the noise if white, that's 1.7 across 250-ish bins?
// So an estimate of the band-limited noise is 68dB.
// pretty respectable.
// handwaving, that's better than a 1ppt meaurement.
// And I could scale up the size of the fft to get a bit better.
// say 1024 samples would get me to > 70dB


