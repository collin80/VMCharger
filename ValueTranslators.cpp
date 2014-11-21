#include <Arduino.h>
#include "ValueTranslators.h"
#include "globals.h"
#include "config.h"
#include "constants.h"

//------------ ensure output power does not exceed other limits
float getAllowedC(float userMaxC) {
  userMaxC=min(userMaxC, absMaxChargerPower/maxOutV );
  userMaxC=min(userMaxC, absMaxChargerCurrent); 
  userMaxC=min(userMaxC, maxMainsC*mainsV/maxOutV);
  
  // check thermal 
  if(normT>=maxHeatSinkT) {
    // start derating
    // map 0-100% derating (relative to the current max current) to 
    // maxHeatSinkT-ABSmaxHeatSinkT heatsink range
    userMaxC=userMaxC*abs(ABSmaxHeatSinkT-normT)/(ABSmaxHeatSinkT-maxHeatSinkT);
    
    if(normT>ABSmaxHeatSinkT) {
      // overheating, stop charger, wait until temp drops enough to restart
      PWM_enable_=0;
      if(LCD_on) myLCD->clrScreen();        

      while(1) {
        sprintf(str, "Cool from %dC", (int)normT);
        printMsg(str, 1000, 0, 1, 0x1F, 0, 0);
        normT=getNormT();
        if(normT<midHeatSinkT) {
          if(LCD_on) myLCD->clrScreen();
          PWM_enable_=1; // restart PWM
          maxOutC1=userMaxC; // full power
          break; // exit cycle when the temp drops enough
        }
      }

    } // ABSmaxHeatSink condition   

  } // end thermal management            

  return userMaxC;
}

void setMaxC(float maxC) {
#ifdef NEG_CSENSE
  // hardware limits in case of opposite direction of the sensor
  Timer1.setPwmDuty(pin_maxC, 1023); // need something more than 3 volts as zero-current output is 2.5V...
#else
  Timer1.setPwmDuty(pin_maxC, 1024./Aref*(V_o_C+k_V_C*maxC));
#endif
}


//============================ current readout functions =====================
//------------ calc current value from ADC value
float readC() {
  return (Aref/1024.*outC_ADC-V_o_C)/k_V_C
#ifdef NEG_CSENSE
          *-1
#endif
          ;
}

//============================ voltage readout functions =====================
// output voltage
float readV() {  
  return (Aref/1024.*
#ifdef DCDC_BUCK // output voltage is lV in case of buck
          outmV_ADC
#else 
          outV_ADC
#endif
          -V_o_bV)*divider_k_bV;
}

// input voltage
float read_mV() {
#ifdef PC817 
  // 3V is a threashold between 120V and 240V - but may require adjustment on a per-unit basis
  if(Aref/1024.*outmV_ADC < 3) return 240;
  return 120;
#endif

  return (Aref/1024.*
#ifdef DCDC_BUCK // input voltage is hV in case of buck
          outV_ADC
#else 
          outmV_ADC
#endif
          -V_o_mV)*divider_k_mV
#ifndef DCinput
          /1.41
#endif
          ;
}
//============================ end voltage readout functions =====================


//====================== temperature readout functions ===========================
// compute the equivalent (normalized) charger temp
byte getNormT() {
  // assume max sink T is 55, max t2 (inductor) is 85 - approx but reasonably close to reality
  // therefore need to rescale t2 by 55/85=0.65
  // BUT ONLY IF WE HAVE THIS SECONDARY MEASUREMENT
#ifdef IND_Temp
  return max(read_T(T_ADC), read_T(T2_ADC)*0.65);
#else
  return read_T(T_ADC);
#endif
}
// master temp readout, using formulas from http://en.wikipedia.org/wiki/Thermistor
byte read_T(unsigned int ADC_val) {
  return (byte)1/( log(1/(1024./ADC_val-1)) /4540.+1/298.)-273; 
}
