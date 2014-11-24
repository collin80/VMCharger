/*
This is the firmware for EMotorWerks Intelligent DC Charging Systems - covering the following products:
* SmartCharge-12000 - a 12kW+ charging system
* QuickCharge-25000 - a 25kW+ charging system
* ISOCharge-20000 - a 20kW+ isolated charging system
* any DC-DC uses of the above

Controller: Arduino Pro Mini 5V (based on a ATmega328P microcontroller)

============================== main operation blocks
------ Startup: 
* check for LCD presence. if none (or the programming button in the programming state), 
  launch in the serial-controlled mode 
* 2 timeouts - one 5 sec for config, one 10 sec for power setting. can be interrupled by any button
* check mains voltage. If 120, limit power to ~1.5kW
------ Charging (CV or CC):
* modulate duty cycle per PID loop calculations running at 250Hz (double the line frequency)
* break when exit condition satisfied or stop / pause commands received

============================== SERIAL COMMAND SYNTAX =================
M,ccc,vvv,sss,E - start charger from 'READY' state with ccc CC point and vvv CV point
                  charger will echo settings. make sure they are what you sent,
                  sss is a checksum = (ccc + vvv) % 1000 
M,001,000,001,E - stop charge
============================== SERIAL STATUS REPORTING ===============
every 100ms or so, the charger will report its status. 
Generally a dump of the critical charging parameters in the following format:
'M,D0,C965,V334,T-68,O1,R0,E' - [D]uty 0%, output [C]urrent 96.5A, output 
[V]oltage 334V, heatsink [T]emp -68C, [O]utput charge 0.1AH, [R]untime 0 minutes

********** HARDWARE MODS REQUIRED to run this firmware on pre-V14 hardware!!! ************
* change RC filter on maxC line (pin 10 Arduino) - ensure C is no more than 1uF
* ensure filter capacitance is less or equal to 0.1uF on all current and voltage senging
*********************************************************************************

This software is released as open source and is free for any personal use.
Commercial use prohibited without written approval from the Author and / or Emotorwerks
Absolutely no warranty is provided and no guarantee for fit for a specific purpose

Original version created Jan 2011 by Valery Miftakhov, Electric Motor Werks, LLC & Inc. 
All rights reserved. Copyright 2014
*/

#include <avr/interrupt.h> 
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include "MemoryFree.h"
#include "config.h"
#include "constants.h"
#include "globals.h"
#include "ConfigStruct.h"
#include "EEPROM_VMcharger.h"
#include "TimerOne.h"
#include "Menu.h"
#include "ValueTranslators.h"

//some of these lines are still left here because I don't yet know or haven't decided what to do with them. FIX IT.
 
struct config_t configuration;

uint8_t PWM_enable_ = 0; //by default disable PWM output until we are really ready for it

extern char str[64]; //temporary storage buffer
float maxOutC = 0., maxOutC1 = 0;
float V_o_mV = V_o_mV0; // need to reassign to non-const as it will be adjusted below
float divider_k_bV = -1.;
float V_o_bV = V_o_bV0; // need to reassign to non-const as it will be adjusted below
float divider_k_mV = -1.;
byte CVreached=0;

// V/A constant for the charger output current sensor 
float V_o_C =
#ifdef OUTC_SENSOR_Allegro_100U
0.6; // allegros are 0.6
#else
2.5; // tamuras are 2.5
#endif

// LCD includes - have to be here in the code as they depend on some switches configured by config.h above
// LCD library for 4D systems display (http://www.4dsystems.com.au/prod.php?id=121)
#ifdef LCD_SPE
  #include "uLCD_144_SPE.h"
  uLCD_144_SPE *myLCD;
#else
#include "uLCD_144.h"
  uLCD_144 *myLCD;
#endif

uint8_t LCD_on=0; // this defines manual vs serial-controlled operation

int cmd[2]={0, 0}; // command variables, used in serial comms

int n = 0; //what the flying hell is n?!?

//===================== charger cycle timers =====================================
uint8_t breakCnt=0;
uint8_t breakCycle=0;

uint32_t timer=0, timer_ch=0, timer_comm=0, timer_irq=0, deltat=0;
uint32_t sec_up = 0;

float mainsV=0, outV=0, outC=0;
float AH_charger=0;

uint8_t charger_run=0;
uint8_t state;
uint8_t normT=0;

int32_t timeOut=0; // in min, 0 means no timeout
float maxOutV=0; // absolute maximum output voltage - will be set later in the code
float maxMainsC=0; // allowed charger power - will be changed in code


//---------------------------------------------------------------------------------------------------------
// as of V13, completely new way to control the charger! proper PID loop and interrupt-based fast ADC
// this was originally driven by the need to meet requirements from the Leaf CHAdeMO protocol
//---------------------------------------------------------------------------------------------------------
// all these have to be ints or very unpleasant wrapping will occur in PID loop 
// having these unsigned has cost EmotorWerks over $1,000 in parts during testing ;-)
int32_t targetC_ADC=0; // this is an ADC reference point for our output current
int32_t outC_ADC_0 = 0, outC_ADC = 0, outV_ADC = 0, outmV_ADC = 0, T_ADC = 0, T2_ADC = 0;
float outC_ADC_f=0;

// ADC interrput handler
// this is always aligned with Timer1 interrupt
// ADC conversions are always done at 4kHz frequency (or every 250uS) so by the next Timer1 interrupt, 
// we should ALWAYS have the result! 
ISR(ADC_vect) { // Analog->Digital Conversion Complete
  byte ul, uh;
  cli(); // disable interrupt until function exit. otherwise nested interrupts...
  ul=ADCL;
  uh=ADCH;
  sei();
  
  unsigned int val= (uh << 8 | ul); // assuming ADLAR=0
  
  // just load things into the variables and exit interrupt - processing all in main loop
  // for most variable, average 2 values offset 180 degrees wrt haversine wave
  // for current measurement, average 16 measurements over 8ms, or one full haversine period
  switch(ADMUX & B00000111) {
   case pin_C:  // this is measured at 2kHz frequency
     if(outC_ADC==0) {
       outC_ADC_f=outC_ADC=val;
     } else {
       // 16 cycles is 8ms here or a full haversine period
       outC_ADC_f=(outC_ADC_f*15+val)/16; // this emulates an RC filter with time constant of ~half of averaged periods
       outC_ADC=int(outC_ADC_f);
     }
     break;   
   // rest of vars measured at 250Hz
   case pin_bV: 
     if(outV_ADC==0) {
       outV_ADC=val;
     } else {
       outV_ADC=(outV_ADC+val)/2;
     }
     break;   
   case pin_mV: 
     if(outmV_ADC==0) {
       outmV_ADC=val;
     } else {
       outmV_ADC=(outmV_ADC+val)/2;
     }
     break;
   case pin_heatSinkT: 
     if(T_ADC==0) {
       T_ADC=val;
     } else {
       T_ADC=(T_ADC+val)/2;
     }
     break;
   case pin_temp2: 
     if(T2_ADC==0) {
       T2_ADC=val;
     } else {
       T2_ADC=(T2_ADC+val)/2;
     }
     break;
   default: break;
  }

} // end ADC interrupt


//---------------- interrupt magic to initiate ADC and calc PID loop ---------------------------
// PID loop setup - see http://en.wikipedia.org/wiki/PID_controller for some definitions
// using only PI part of it here
// parameter approximations ------------
// all constants below are effectively in 0.0001 units for the formula
// at 10-bit duty counter, 250Hz loop speed and 1000 unit range
// Example: 50A target ramp from zero, measured with a 50A bidir sensor: error is ~300
//          ramp rate is pids_Kp * 8 duty pts / sec (pids_Kp * 300 / 10000 duty points in one cycle (~4ms))
// for CHAdeMO unit with 50A max C, ramp to 50A in 2 seconds requires pids_Kp>60 (assuming full duty sweep would be required)
// OTOH, typical single-stage charger's stiffness is 10-20A per 10 duty points 
// so we don't want to be making changes of more than 10 duty points per cycle
// which corresponds to pids_Kp<330
// so the meaningfull range is probably between 50 and 300
// our motor controller has Kp=3200, Ki=30, Kd=0 
//----------------------- tuning charger PID:
// Zieglerâ€“Nichols method: the Ki and Kd gains are first set to zero. 
// The P gain is increased until it reaches the ultimate gain, Ku, at which the output of the loop 
// starts to oscillate. Ku and the oscillation period Pu are used to set the gains as shown:
// Control Type	Kp	Ki	        Kd
//    P    	0.50Ku	-	        -
//    PI	        0.45Ku	1.2Kp / Pu	-
//    PID	        0.60Ku	2Kp / Pu	KpPu / 8
// for this charger:
// on 330V pack (LiFePo4, milli-ohm total IR), at Kp=1000, see oscillations at Hz) - hence 
// setting Kp=, Ki=

int32_t pids_Kp = 0;
long pids_err=0, pids_perr=0, pids_i=0, pids_d=0; // all have to be signed longs in order to not screw up pid calcs
long deltaDuty=0, milliduty=0; // have to be signed
byte tickerPWM=0; // short counter used only to skip cycles
byte tickerPWM1=0; // counter counting unskipped cycles - ok to overwrap

// called on overflow of Timer1 - called every 'period' uS (20 kHz by default)
// overflow with TimerOne library means we are in the center of the PWM cycle (TimerOne is phase correct)
void sampleInterrupt() {
  // trigger actual work only on every Nth period
  tickerPWM++;
  
  // prescale is defined based on period to make constant frequency of work below
  if(tickerPWM < MEASFREQPWMPRESCALE) return; 
  // prescaler is calculated at startup so that we always end up with ~4kHz frequency here
  // therefore, every ADC conversion has 250 microseconds - which should be ok given that ADC on ATMega328P takes 100us

  tickerPWM=0;
  tickerPWM1++; // this counts at lower frequency
    
  ADMUX &= B11111000; // reset the channel to zero

  // then current is measured every second cycle - or at ~2kHz frequency
  if(tickerPWM1 & 0x1) {
     ADMUX |= pin_C;
     ADCSRA |= B11000000; // manually trigger next one
  } else {
    // Every parameter is measured every 16 cycles => 250 Hz measurement frequency for every variable
    // PID loop runs at the same frequency, as well    
    switch(tickerPWM1/2 & 0x7) {  
       // case set below is MISSING 0,4,7 - available for other sensors
       case 0: {
         // average outC
         if(fabs(outC)<1.) outC=readC(); // 
         outC=(outC*float(AVGCycles-1)+readC())/AVGCycles; 
         break;
       }
       case 1: {
         ADMUX |= pin_bV;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 2: {
         ADMUX |= pin_mV;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 3: {
         ADMUX |= pin_heatSinkT;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 4: {
         // average outV
         if(fabs(outV)<1.) outV=readV();
         outV=(outV*float(AVGCycles-1)+readV())/AVGCycles;
         break;
       }
       case 5: {
         ADMUX |= pin_temp2;
         ADCSRA |= B11000000; // manually trigger next one
         break;
       }
       case 6: {
          //====================   PID loop   ====================
          // remember that targetC is a 10-bit ADC reference point that we are trying to keep - NOT the actual current!
          pids_err = targetC_ADC - outC_ADC; 
  #ifdef NEG_CSENSE
          pids_err *= -1; // the current signal (and hence the error sign) runs in a different direction
  #endif        
          
          deltaDuty = pids_Kp * pids_err;
      
          pids_i += pids_err;
          deltaDuty += pids_Ki * pids_i;
      
          pids_d = pids_err - pids_perr;
          pids_perr = pids_err;
          deltaDuty += pids_Kd * pids_d;
          //==================== end PID loop ====================
          
          // protect against overpowering
          if( (deltaDuty>0) && (outC > 1.1*maxOutC) ) deltaDuty=0;
          
          milliduty += deltaDuty;
          if(milliduty < 0) {
            milliduty=0;
            // stop accumulation
            pids_i=0;
          }
          if(milliduty > MAXDMILLIDUTY) {
            milliduty=MAXDMILLIDUTY;
            // stop error accumulation
            if(pids_i>0) pids_i=0;
          }
  
          // immediate protection from overvoltage - zero out duty
          // this also stops any term's accumulation before PWM_enable_ is turned on (e.g. before charger start)
          if( (PWM_enable_ == 0) || (outV > 1.05*maxOutV) ) {
            milliduty=0;
            pids_i=0; // need to stop accumulation, as well
            // this is an emergency so stop all PWM
            PWM_enable_=0;
          }
          
          Timer1.setPwmDuty(pin_PWM, milliduty/10000); 
  
          break;  
       }

       default: break;

    } // end switch
  
  } // end if(tickerPWM1 & 0x1)
  
}  // end timer interrupt


//-------------------------------------------- START MAIN CODE ---------------------------------------------
void setup() {
  // digital inputs
  pinMode(pin_pwrCtrlButton, INPUT);
  pinMode(pin_pwrCtrl2Button, INPUT);
  pinMode(pin_J1772, INPUT);
  pinMode(pin_BMS, INPUT);
  pinMode(pin_DELTAQ, INPUT);

  // set output digital pins
  pinMode(pin_outrelay, OUTPUT);
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_maxC, OUTPUT);
  pinMode(pin_EOC, OUTPUT);
  pinMode(pin_fan, OUTPUT);
  pinMode(pin_inrelay, OUTPUT);
  
  // setup ADC
  ADMUX = B01000000;  // default to AVCC VRef, ADC Right Adjust, and ADC channel 0 (current)
  ADCSRB = B00000000; // Analog Input bank 1
  // ADC enable, ADC start, manual trigger mode, ADC interrupt enable, prescaler = 128 (3 bits in the end)
  // standard prescaler is 128 resulting in 125kHz ADC clock. 1 conversion takes 13 ADC cycles = 100uS using standard prescaler
  // 64 prescaler results in ~50uS conversion time
  ADCSRA = B11001111; 
  
  // setup timer - has to be before any ADC readouts
  Timer1.initialize(period); 
  Timer1.pwm(pin_PWM, 0); // need this here to enable interrupt
  Timer1.pwm(pin_maxC, 0); // need this here to enable interrupt
  Timer1.attachInterrupt(&sampleInterrupt); // attach our main ADC / PID interrupt
  delay(50); // allow interrupts to fill in all analog values 

  //=================================== finalize init of the sensors =============================
  // reset voltage dividers to account for the input resistance of ISO124
  divider_k_mV=upperR0_mV/lowerR_mV;
  divider_k_bV=upperR0_bV/lowerR_bV;
  //=============================== END finalize init of the sensors =============================

  //================= initialize the display ===========================================
#ifdef LCD_SPE
  *myLCD=uLCD_144_SPE(9600);
#else
  *myLCD=uLCD_144(9600);
#endif
  //================= finish display init ==============================================
  
  // check if the display started / is present
  // if not present, we will assume that the charger is controlled by serial data instead
  LCD_on=myLCD->isAlive();

  //==================================== ONE-TIME CONFIG =================================
  // check if needed to go into config 
  byte forceConfig=255; // default is 255 - has to be different from 0 or 1
  EEPROM_readAnything(0, configuration);
  // reset configuration if the green button is pressed at charger start
  // on first connection, do zero cal of mainsV, as well
  if(configuration.CC<=0 || digitalRead(pin_pwrCtrl2Button)==1) {
    forceConfig=1; // first time running the charger after assembly
    configuration.CV=350;
    // set the rest of the vars
    configuration.Vcal=0;
    configuration.Vcal_k=1.; // prefill the calibration with unity so we don't get zero readings if calibration menu is skipped
    configuration.mVcal=0;
    configuration.Ccal=0;
  }
  
  const byte STATE_DONE = 0xff;
  const byte STATE_CV = 0x1;
  const byte STATE_CELLS = 0x2;
  const byte STATE_CAPACITY = 0x4;
  const byte STATE_CALIBRATE = 0x5; // sensitivity calibration only. zero point calibration done automatically on power-on
  state = STATE_CV;
    
  if(LCD_on) {  
    myLCD->clrScreen();
    myLCD->setOpacity(1);
  } else {
    state=STATE_DONE; // skip config altogether if no LCD
    // reset serial to faster speed
    Serial.end();
    Serial.begin(serialspeed);
  }    
  
  while(state != STATE_DONE)
  {
    switch(state)
   {
     case STATE_CV:
       // if config is not forced, just timeout and send to end of config. Else, wait until button press
       if(forceConfig==255) {
         printClrMsg(MSG_THX, 50, 0, 0x3f, 0);
         forceConfig=BtnTimeout(5, 7); // this will return 0 if no button pressed; 1 otherwise; 5 seconds, line #7
       }
       if(forceConfig==0) {
         state=STATE_DONE;
       } else { // forceConfig=1 here
         myLCD->clrScreen();
         printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CV);
         configuration.CV = DecimalDigitInput3(configuration.CV); 
         state = STATE_CELLS;       
       }
       break;
     case STATE_CELLS:
       printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_NCELLS);
       configuration.nCells = DecimalDigitInput3(configuration.nCells); 
       state = STATE_CAPACITY;
       break;
     case STATE_CAPACITY:
       printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CAPACITY);
       configuration.AH = DecimalDigitInput3(configuration.AH); 
       state = STATE_CALIBRATE;       
       break;
     case STATE_CALIBRATE:
       // output current zero calibration - this assumes that there is no load on startup 
       // this is especially important for PFCdirect units which should not have anything plugged into charger output at this point!
       outC_ADC_0=outC_ADC; // ADC reference
       outC=readC();  
#ifdef NEG_CSENSE
       configuration.Ccal=-outC*k_V_C;
#else
       configuration.Ccal=outC*k_V_C;
#endif

       // prep for output voltage zero calibration
       // this will generally NOT work on PFCdirect units as there is always voltage on the output
       // to calibrate at the factory / right after build, power 12V ONLY and follow through calibration
       outV=readV();
       sprintf(str, "Drain %dV, BTN", int(outV));  
       printMsg(str, 0, 0, 0, 0x1f, 0x3f, 0x00);
       while(!(digitalRead(pin_pwrCtrlButton) || digitalRead(pin_pwrCtrl2Button)));
       outV=readV(); // re-read after discharge

       // now actual zero cal
       if(fabs(outV)<40) { // if too far off, fault out
         // output voltage calibration
         configuration.Vcal=outV/divider_k_bV; 
         V_o_bV+=configuration.Vcal; // this needs to be adjusted HERE because we are calling readV() again below for sensitivity calibration
         printConstStr(0, 5, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CAL0);
         delay(1000);
       }
       
       // now calibrate voltage sensor slope
       // first, double-check we have reset to zero point
       // for PFCdirect units, this will only work if ONLY 12V is powered up, no main AC connected!
       outV=readV(); // get the readings with zero-point already calibrated
       if(fabs(outV)<3) { // should be pretty tight after zero calibration
         // this is a good time to also do mains calibration - assuming that on the very first power-up and forced config in general
         // we have zero input AC voltage
         if(forceConfig==1) {
           mainsV=read_mV();
           // only recal if not too far from truth
           if(mainsV<30) {
             configuration.mVcal=mainsV/divider_k_mV;
           } else {
             configuration.mVcal=0;
           }
         }
         myLCD->clrScreen();
         printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CAL1); // this asks to connect the battery
         delay(1000); // to avoid reading same button state as in prev step
         while(1) {
           outV=readV();
           if(digitalRead(pin_pwrCtrlButton) || digitalRead(pin_pwrCtrl2Button))  break;
           if(outV>20) { // loop until battery not connected
             // battery has been connected, now need to wait until voltage stabilizes
             // in units with 390R precharge, time constant is up to 4 seconds
             // we need to wait for 4 constants (15 seconds), then close relay, measure, and open relay again
             delay(15000); // let settle
             digitalWrite(pin_outrelay, HIGH);
             delay(500);
             outV=readV(); // read settled voltage
             digitalWrite(pin_outrelay, LOW);
             // calibrate
             printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CAL2);
             // calibration routine here - if actual voltage > shown, REDUCE the constant
             configuration.Vcal_k=DecimalDigitInput3(int(outV))/outV;
             break; // from while() loop
           }
         }
       }
       
       state = STATE_DONE;
       break;

     default: break;
   } 
  }

  // parameters calculated from config variables go here
  // adjust core sensor constants
  V_o_bV=V_o_bV0+configuration.Vcal;
  V_o_mV=V_o_mV0+configuration.mVcal;
  V_o_C+=configuration.Ccal;
  divider_k_bV*=configuration.Vcal_k; 
  
  // write out the configuration to EEPROM for next time
  EEPROM_writeAnything(0, configuration);

#ifdef DEBUG1
  Serial.print("MFP: ");
  Serial.println(MEASFREQPWMPRESCALE);
#endif
}
  

void loop() {  
  // ---------------real loop()
  byte x=255; // default, has to be different from 0 or 1
  int J1772_dur;

  mainsV=read_mV();
  outV=readV();
  
  maxOutV=float(configuration.CV)/100*configuration.nCells;

  // run charger if: 
  //         (1) charger has NOT been run yet in this cycle, or 
  //         (2) has been run over a week ago
  //         (3) green button is pressed to override
  if(LCD_on==0 || digitalRead(pin_pwrCtrl2Button)==HIGH || charger_run==0) {
      //----------------------------
      // run state machine:
      const byte STATE_TOP_MENU = 0x00;
      const byte STATE_CONFIG_PWR = 0x01;
      const byte STATE_CONFIG_TIMER = 0x02;
      const byte STATE_CHARGE = 0x04;
      const byte STATE_WAIT_TIMEOUT = 0x05;
      const byte STATE_SERIALCONTROL = 0x10;
      const byte STATE_SHUTDOWN = 0xff;
      if(!LCD_on) {
        // drop us directly into a serial control loop
        state=STATE_SERIALCONTROL;
      } else {
        state = STATE_WAIT_TIMEOUT;
      }
      if(configuration.CC<=0) state=STATE_CONFIG_PWR;
      
      while(state != STATE_SHUTDOWN)
      {
        // reload voltages
        mainsV=read_mV();
        outV=readV();
        
        if(LCD_on) {
          myLCD->clrScreen();
          printConstStr(0, 6, 2, 0x1f, 0x3f, 0, MSG_LCD_PARAMS);
          sprintf(str, "IN: %dV, %luA", int(mainsV), configuration.mainsC); 
		  myLCD->printStr(1, 7, 2, 0x1f, 0x3f, 0, str);
          sprintf(str, "OUT: %dV, %luA", int(outV), configuration.CC); 
		  myLCD->printStr(1, 8, 2, 0x1f, 0x3f, 0, str);
          sprintf(str, "T-OUT: %li min", timeOut); 
		  myLCD->printStr(1, 9, 2, 0x1f, 0x3f, 0, str); 
        }
        
        //======================== MAIN STATE MACHINE ======================
        switch(state)
        {
       
        case STATE_WAIT_TIMEOUT:
          printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_CFG);
          
          // check J1772
          J1772_dur=pulseIn(pin_J1772, HIGH);
          if(J1772_dur>50) { // noise control. also, deals with the case when no J1772 signal present at all
            configuration.mainsC=0.06*J1772_dur+3; // J1772 spec - every 100uS = 6A input - this will work up to 48A
            if(LCD_on) {
              sprintf(str, "IN: %dV, %luA", int(mainsV), configuration.mainsC); myLCD->printStr(1, 7, 2, 0x1f, 0x3f, 0, str);
            }
          }
          
          x=BtnTimeout(10, 3);

          if(x == 1) state = STATE_TOP_MENU; // some button was pressed
          if(x == 0) // nothing pressed
           { 
            state = STATE_CHARGE;
           }
          break;
       
        case STATE_TOP_MENU:
          printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_TOPMENU);
          x=MenuSelector2(configMenuLen, configMenu);
          switch(x)
          {
            case 0: state = STATE_CHARGE; break;
            case 1: state = STATE_CONFIG_PWR; break;
            case 2: state = STATE_CONFIG_TIMER; break;
            default: break;
          }
          break;

        case STATE_CONFIG_PWR:
          printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_INC);      
          configuration.mainsC = DecimalDigitInput3(configuration.mainsC); 
          printConstStr(0, 0, 2, 0x1f, 0x3f, 0x00, MSG_LCD_OUTC);      
          configuration.CC = DecimalDigitInput3(configuration.CC); 
          state = STATE_TOP_MENU;
          break;

        case STATE_CONFIG_TIMER:
            // now set the timer using the same button       
            printConstStr(0, 0, 2, 0x1f, 0x3f, 0x1f, MSG_LCD_TOUT);
            timeOut=DecimalDigitInput3(0); 
           state = STATE_TOP_MENU;
           break;

        case STATE_SERIALCONTROL:
			if (getSerialCmd()) {
				state=STATE_CHARGE;
			} else {
				delay(200); // wait a bit and do another check for a command - cannot wait too long due to QC timing. 
			}
			break;
    
         case STATE_CHARGE: 
           // cannot delay from here to charging function QC operation requires quick ramp after the command
           // write out the configuration to EEPROM for next time
           EEPROM_writeAnything(0, configuration);
  
           maxMainsC=configuration.mainsC; 
           mainsV=read_mV(); // for power adjustments
#ifdef drop110power       
           if(J1772_dur<50) { // but only if no J signal
             // curb power on 110VAC
             if(mainsV<160) { 
               maxMainsC=min(configuration.mainsC/2, 9.); // equivalent 15A from 110VAC // DEBUG
             }
           }
#endif 

            maxOutC=getAllowedC(configuration.CC); 
            
            //  set max hardware current protection to the fixed absMaxChargeCurrent value
            setMaxC(peakMaxC*getAllowedC(absMaxChargerCurrent)); 
            
            timer_ch=millis(); // set the timer          
            AH_charger=0; // reset AH counter
           
// zero motorcycles special now
#ifdef DELTAQ
            while(digitalRead(pin_DELTAQ)); // wait until the pin is pulled down by the BMS
#endif

            // reset the EOC pin (this pin is active LOW so reset means setting to HIGH) 
            // high means charging is commencing. this will be pulled down by the charger when charge ends
            // this also feeds a closed-loop BMS
            digitalWrite(pin_EOC, HIGH); 
              
            //========================== MAIN RUN CHARGER FUNCTION=======================
            // by this point, at least 15sec have passed since AC connection
            //                and at least 10sec since battery connection
            //                therefore, all caps should be pre-charged => close relays
            //   (note: this requires precharge resistors across relays - MAX of 300R for inrelay
            //          and MAX of 1k for outrelay. >3W power rating. Place small 1000V diode in
            //          series with the outrelay resistor - anode to battery - to avoid precharge on
            //          reverse polarity connection)     
            digitalWrite(pin_inrelay, HIGH);
            digitalWrite(pin_outrelay, HIGH);
            
            // check for invalid sensor configs
            // most dangerous is disconnection of the current sensor
            // generally will manifest itself by non-zero current reading while duty is zero 
            // (which it should be at this point)
            // 10A is a lot of margin for that
            if(fabs(readC())>10) {
              return;
            }
  
            // CC-CV profile, end condition - voltage goes to CV and current goes to X% of CC
            runChargeStep();
            PWM_enable_=0; // HAS to be here to ensure complete stop on any condition
  
            // make sure everything is off
            digitalWrite(pin_inrelay, LOW);
            digitalWrite(pin_outrelay, LOW);
            digitalWrite(pin_fan, LOW);    
            digitalWrite(pin_EOC, LOW); // active low
            //==================== charger routine exited ===============================
  
            printClrMsg(MSG_DONE, 500, 0x1f, 0x3f, 0); 
            sprintf(str, "%dAH", int(AH_charger)); 
            if(LCD_on) {
              myLCD->printStr(0, 6, 2, 0x1f, 0x3f, 0x1f, str);      
              charger_run=1; // charger has run this mains cycle...
              state = STATE_SHUTDOWN; //STATE_TOP_MENU;   
            } else {
              EMWserialMsg(str);
              state = STATE_SERIALCONTROL; // ready for next run   
            }
  
            break; 

          default: break;
        }
        //=========================== END MAIN STATE MACHINE
      }
    }
  
}


//-------------------------------- main charger routine ------------------------------------------------
int runChargeStep() {
  pids_Kp=pids_Kp_FAST; // start with fast PID - will be changed to slow when we see some current
  
  maxOutC1=maxOutC;
  
  CVreached=0; // status of CV state
  
  // reset V,C readings - otherwise averaging gets screwed up really badly
  outC=0; 
  outV=0; 
  outC_ADC_0=0, outC_ADC=0, outV_ADC=0, outmV_ADC=0, T_ADC=0, T2_ADC=0;

  if(LCD_on) {
    myLCD->clrScreen();
    sprintf(str, "CC=%dA, CV=%dV", int(maxOutC), int(maxOutV)); 
    myLCD->printStr(0, 4, 2, 0x1f, 0x3f, 0x1f, str);      
    delay(5000);
    myLCD->clrScreen();
  } else {
    // machine-readable
    // this assumes that only CC commands will be issued to the charger via serial
    sprintf(str, "I:%d,%d,%d", int(configuration.AH*min_CV_Crating), int(maxOutC), int(maxOutV)); 
    EMWserialMsg(str);
  }

  // reset timers - for AH metering and serial comms
  timer=millis(); // this will be reset every cycle below after AH delta is calculated
  timer_comm=timer;
  
  // turn on PWM output
  PWM_enable_=1;
  
  //============================================== MAIN CHARGER LOOP =============================================
  
  while(1) {
    // NOTE THAT outC / outV readings are all set in the interrupts

    // track targetC to maxOutC1
    targetC_ADC=1024*(k_V_C*maxOutC1
#ifdef NEG_CSENSE
          *-1
#endif
          +V_o_C)/Aref;

    if(!LCD_on) {
		pollSerial(); //reads in serial data if there is some waiting
    }
    
    // process serial commands and print out status only every 50ms or so
    // in LCD mode, this just increments the cycle counter for proper timing of the LCD printout
    if(millis()-timer_comm > stepDelay) { 
      n++; 
      timer_comm=millis();
      
      normT=getNormT();
  
  #ifdef DEBUG2
      Serial.print("free RAM: "); Serial.println(freeRam());
      Serial.print("       -outC_ADC="); Serial.println(outC_ADC);
      Serial.print("            -outC="); Serial.println(int(outC));
      Serial.print("            -maxOutC="); Serial.println(maxOutC);
      Serial.print("            -targetC="); Serial.println(targetC_ADC);
      Serial.print("       -outV_ADC="); Serial.println(outV_ADC);
      Serial.print("       -outmV_ADC="); Serial.println(outmV_ADC);
      Serial.print("       -T_ADC="); Serial.println(T_ADC);
      Serial.print("            -normT="); Serial.println(normT);
      Serial.print("       -duty="); Serial.println(milliduty/10000);
  //    Serial.print(""); Serial.println();
  #endif
    
      // if in Serial mode, check for commands here
      if(!LCD_on) {
		if (!processSerial()) return 0;
	  } else {        
        // slow voltage control cycle here. AT Cstep=0.5A default, we are ramping down at ~5A/second
        // this may not be enough to avoid a bit of overvoltage beyond CV
        // need to do it ONLY for non-serial control as in serial control the charger is a slave
        if(outV > maxOutV) {
          CVreached=1;
          maxOutC-=Cstep;
          if(maxOutC<0) maxOutC=0;
        }    
        // recalc maxOutC1 - this will account for temp derating
        maxOutC1=getAllowedC(maxOutC);
        delay(30); // a delay equivalent to non-LCD execution time
      }
    }
    
    //------------------------------------------------ print out stats ----------------------------------------
    // but only every few hundred cycles. defaults: measCycle_len=20, stepDelay=30
    if(n>measCycle_len) {
      n=0;

      // timer
      sec_up=(unsigned int)1.*(millis()-timer_ch)/1000;

      // check for break conditions 
      // mask the first few seconds
      if(sec_up>CV_timeout && CVreached && maxOutC1 < configuration.AH*min_CV_Crating) {
        breakCycle=1;
      } else {
        breakCycle=0; // reset 
      }
      // do we REALLY need to break?
      if(breakCycle) {
        breakCnt++;
        if(breakCnt>stopCycles) {
          printClrMsg(MSG_NORMEXIT, 5000, 0, 0x3f, 0);
          return 0; 
        }
      } else {
        breakCnt=0;
      }    
      
      // AH meter
      AH_charger+=outC*int(millis()-timer)/1000/3600;
      timer=millis();
      
      // check HVC signal from BMS
      if(digitalRead(pin_BMS)==LOW
#ifdef DELTAQ
            || digitalRead(pin_DELTAQ)
#endif
      ) { // active LOW (so you need to pull up by connecting miniBMS loop to EOC signal)
        // BMS commanding charger to stop
        // noise protection - ensure signal stays on for 100ms or so
        delay(100);
        if(digitalRead(pin_BMS)==LOW
#ifdef DELTAQ
            || digitalRead(pin_DELTAQ)
#endif        
        ) {
          // this is for real
          printClrMsg(MSG_BMSSTOP, 5000, 0x1f, 0x3f, 0);
          return 0; 
        }
      } 
            
      // check the timer
      if(timeOut>0 && (millis()-timer_ch)/60000>timeOut) {
        // timer run out
        printClrMsg(MSG_TIMEOUT, 5000, 0x1f, 0x3f, 0);
        return 0; 
      }
      
      //==================== print all parameters
      // print here only if LCD is on - otherwise print in faster loop over Serial
      if(LCD_on) {
        printParams(outV, outC, normT, AH_charger, maxOutC1, maxOutV);
      }
      
      if(outV < -10 || outC < -10) {
        // sensor polarity problems. abort
        printClrMsg(MSG_SENSEERROR, 300, 0, 0x3f, 0);
        return 1; // full stop
      }
      
      // check if need to stop - RED button pressed? - both in LCD and non-LCD modes
      if(digitalRead(pin_pwrCtrlButton)==HIGH) return 0; 
      
#ifdef CHECKMAINS         
      // check mains
      if(read_mV()<minMains) {
        delay(2000);
        printClrMsg(MSG_LOSTIN, 5000, 0x1f, 0x3f, 0);
        return 1; // error
      }
#endif
      
    }  // end measCycle loop    
    
  }; //======================================== END MAIN CHARGER LOOP ===================================

  return 0;
} // end runChargeStep()

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void setMaxC(float maxC) {
#ifdef NEG_CSENSE
  // hardware limits in case of opposite direction of the sensor
  Timer1.setPwmDuty(pin_maxC, 1023); // need something more than 3 volts as zero-current output is 2.5V...
#else
  Timer1.setPwmDuty(pin_maxC, 1024./Aref*(V_o_C+k_V_C*maxC));
#endif
}