/*
 * sys_io.cpp
 *
 * Handles the low level details of system I/O
 *
Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

some portions based on code credited as:
Arduino Due ADC->DMA->USB 1MSPS
by stimmer

*/ 

#include "adc.h"

#undef HID_ENABLED

#define NUM_ANALOG	8
#define ADC_BUFFS	4
#define NUM_ADC_SAMPLES	256 / NUM_ANALOG

volatile int bufn,obufn;
volatile uint16_t adc_buf[ADC_BUFFS][256];
uint16_t adc_values[NUM_ANALOG];

/*
Initialize DMA driven ADC and read in gain/offset for each channel
*/
void setup_adc() {
  int i;

  setupFastADC();

  //requires the value to be contiguous in memory
  for (i = 0; i < NUM_ANALOG; i++) {
	adc_values[i] = 0;
  }
}

/*
get value of one of the analog inputs
Uses a special buffer which has smoothed and corrected ADC values. This call is very fast
because the actual work is done via DMA and then a separate polled step.
*/
uint16_t getAnalog(uint8_t which) {
    uint16_t val;
	
    if (which >= NUM_ANALOG) which = 0;

	return adc_values[which];
}


/*
When the ADC reads in the programmed # of readings it will do two things:
1. It loads the next buffer and buffer size into current buffer and size
2. It sends this interrupt
This interrupt then loads the "next" fields with the proper values. This is 
done with a four position buffer. In this way the ADC is constantly sampling
and this happens virtually for free. It all happens in the background with
minimal CPU overhead.
*/
void ADC_Handler(){     // move DMA pointers to next buffer
  int f=ADC->ADC_ISR;
  int nBuf = 0;
  if (f & (1<<27)){ //receive counter end of buffer
   bufn=(bufn+1) % ADC_BUFFS;
   nBuf = (bufn + 1) % ADC_BUFFS; //this is the next buffer so we need to increment yet again
   ADC->ADC_RNPR=(uint32_t)adc_buf[nBuf];
   ADC->ADC_RNCR=256;  
  } 
  adc_poll(); //not the best idea. the goal is to not run this function within the int handler
}

/*
Setup the system to continuously read the proper ADC channels and use DMA to place the results into RAM
Testing to find a good batch of settings for how fast to do ADC readings. The relevant areas:
1. In the adc_init call it is possible to use something other than ADC_FREQ_MAX to slow down the ADC clock
2. ADC_MR has a clock divisor, start up time, settling time, tracking time, and transfer time. These can be adjusted
*/
void setupFastADC(){
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST); //just about to change a bunch of these parameters with the next command

  /*
  The MCLK is 12MHz on our boards. The ADC can only run 1MHz so the prescaler must be at least 12x.
  The ADC should take Tracking+Transfer for each read when it is set to switch channels with each read

  Example:
  Now, if the divider is 50x then 1M / 50 = 20,000 clock rate. If we set our adc to use 12 ticks per reading then
  that is 1666 reads per second. There are 8 ADC channels being read so each channel gets about 208 reads per second
  or approximately one reading every 5-6ms. 
  */
  ADC->ADC_MR = (1 << 7) //free running
              + (24 << 8) //50x MCLK divider ((This value + 1) * 2) = divisor
			  + (1 << 16) //8 periods start up time (0=0clks, 1=8clks, 2=16clks, 3=24, 4=64, 5=80, 6=96, etc)
              + (1 << 20) //settling time (0=3clks, 1=5clks, 2=9clks, 3=17clks)
              + (4 << 24) //tracking time (Value + 1) clocks
              + (2 << 28);//transfer time ((Value * 2) + 3) clocks

  ADC->ADC_CHER=0xFF; //enable A0-A7

  NVIC_EnableIRQ(ADC_IRQn);
  ADC->ADC_IDR=~(1<<27); //dont disable the ADC interrupt for rx end
  ADC->ADC_IER=1<<27; //do enable it
  ADC->ADC_RPR=(uint32_t)adc_buf[0];   // DMA buffer
  ADC->ADC_RCR=256; //# of samples to take
  ADC->ADC_RNPR=(uint32_t)adc_buf[1]; // next DMA buffer
  ADC->ADC_RNCR=256; //# of samples to take
  bufn=obufn=0;
  ADC->ADC_PTCR=1; //enable dma mode
  ADC->ADC_CR=2; //start conversions

}

//polls	for the end of an adc conversion event. Then processe buffer to extract the averaged
//value. It takes this value and averages it with the existing value in an 8 position buffer
//which serves as a super fast place for other code to retrieve ADC values
void adc_poll() {
	if (obufn != bufn) {
		uint32_t tempbuff[8] = {0,0,0,0,0,0,0,0}; //make sure its zero'd
	
		//the eight or four enabled adcs are interleaved in the buffer
		//this is a somewhat unrolled for loop with no incrementer. it's odd but it works
		for (int i = 0; i < 256;) {	   
			tempbuff[7] += adc_buf[obufn][i++];
			tempbuff[6] += adc_buf[obufn][i++];
			tempbuff[5] += adc_buf[obufn][i++];
			tempbuff[4] += adc_buf[obufn][i++];
			tempbuff[3] += adc_buf[obufn][i++];
			tempbuff[2] += adc_buf[obufn][i++];
			tempbuff[1] += adc_buf[obufn][i++];
			tempbuff[0] += adc_buf[obufn][i++];
		}

		for (int j = 0; j < 8; j++) {
			adc_values[j] += (tempbuff[j] >> 5);
			adc_values[j] = adc_values[j] >> 1;
		}
  
		obufn = bufn;    
	}
}


