/*
 * constants.h
 *
 * Defines the global / application wide constants
 *

 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <avr/pgmspace.h>
#include <Arduino.h>

//Don't change this hard coded value!
const uint32_t MAXDMILLIDUTY = 9700000; // to run precise PID loop   

const int PWMFreq = 1000000 / period;

const unsigned int linefreq = 60; // 60Hz in the US - for best performance, set this to your country's line frequency!
// scaler from PWM frequency. There is a method to the madness here - basically we want to be sampling things at 4x the line
// frequency and then use two adjacent readings to produce an average. This cancels out most of the 120Hz ripple from the readings
// in the sampling interrupt, every variable is sampled only every 1/16th period, hence the 16 divider below
/* 
CK - The stated goal of the below line is to scale down to 4KHz at the PWM interrupt. The previous way this was calculated
left something to be desired. This has all been corrected to return the proper value for all input frequencies.
As an example:  

PWM period of 130us and power at 60Hz -> (((100 * 1000 / 130) * 1000) / (240) / 16 / 100) - 1 = 1
A value of 1 will cause the PWM interrupt to run the code every other time. A period of 130us is around 8KHz so every other
time is 4Khz.

Another example:
PWM period of 50us and power at 60Hz -> (((100 * 1000 / 50) * 1000) / (240) / 16 / 100) - 1 = 4
A value of 4 will cause the PWM interrupt to run the code every 5th cycle through. A period of 50 is 20KHz. 20/5 = 4khz which is
the proper speed.
*/
const int32_t MEASFREQPWMPRESCALE = (((100l * 1000l / period) * 1000l) / (linefreq * 4l) / 16l / 100l) - 1;

//---------------- pin-out constants ----------------
//========== analog pins
const uint8_t pin_C = 0; // output current pin
const uint8_t pin_bV = 1; // output / battery voltage pin
const uint8_t pin_heatSinkT = 2; // charger heatsink temp - for thermal derating 
const uint8_t pin_12Vsense = 3; // implementing undervoltage protection
const uint8_t pin_temp2 = 4; // 4 - spare prewired as temp input
const uint8_t pin_mV = 5;
// const uint8_t pin_mC=7; // will only work in V12+ control boards (June 2013). needed only for full digital PFC control
//========== digital pins
// 0/1 reserved for serial comms with display etc
const byte pin_pwrCtrlButton = 2; // this is wired to the button (used for menu step)
const byte pin_pwrCtrl2Button = 3; // this is wired to the button2 (used for menu select)
const byte pin_inrelay = 4; // precharges input caps - normally pin 4, in some units pin 6 running fan relay
const byte pin_outrelay=4; // output relay - this is needed for CHAdeMO; same pin to free up pin 5 for dual-direction units
const byte pin_dirCtrl=5; // control of the direction of power flow - LOW=BUCK, HIGH=BOOST
const byte pin_DELTAQ = 6; // deltaQ pin
const byte pin_J1772 = 7; // J1772 pilot input. 1k is hardwired on V14+ pcbs so J1772 will power on on connect
const byte pin_fan = 8; // fan control - this is pin4 in all kits shipped before Mar '12
const byte pin_PWM = 9; // main PWM pin

// max current reference voltage (using PWM) -  was 6 in the V13 pcb (kits shipped before March 2012)
// now moved to pin 10 so that we can use higher PWM frequency 20kHz PWM
const uint8_t pin_maxC = 10;

const uint8_t pin_EOC = 12; // end-of-charge output (see pinout diagram) - pulled low when charge is complete

// end-of-charge input from BMS. Pull low / disconnect from positive TTL signal to activate
//     (normallly will be realized via connecting NC BMS loop between this pin and EOC pin (or +5V)
const uint8_t pin_BMS = 13;
//---------------- END PINOUTS -----------------------

//---------------- MAX CURRENTS
// absolute maximum average output current (used in CV mode) - leave at 0 here - will be set via power menu
const float min_CV_Crating = 0.05; // wait until the current goes to XC (use values from your battery's datasheet)
const float Cstep = 0.5; // how quickly the current tapers off in CV step - A / second. cannot be too high to prevent false stops

// ratio between the average and max current in the inductor before overcurrent kicks in. 1.8 is a good value
// as it allows some ripple and some margin on top of that
const float peakMaxC = 2.0; 

#ifdef debugpower // increase power limits for testing runs - careful!
const float absMaxChargerCurrent = 300; // 300A...
const float absMaxChargerPower = 50000; // 50kW...
#else
#ifdef MCC100A
const float absMaxChargerCurrent = 99; // 99A (need to be a 2-digit number!) rating with high-current output toroid inductor
#else
const float absMaxChargerCurrent = 70; // 70A default rating with new toroid inductors
#endif

#ifdef PFCdirect
const float absMaxChargerPower = 25000; // 25kW rating for PFCDirect units with new 5-6" toroid inductors
#else
const float absMaxChargerPower = 12000; // 12kW rating for regular units with new 4" toroid inductors
#endif
#endif

// input currents (used only for DC-DC units (if DCinput switch is active)
#ifdef DCinput
const float MAXinputC = absMaxChargerCurrent;
#endif

//------------- THERMAL DERATING OF CHARGER 
// for now, simple protection by pausing charger until cooldown to certain temp
// note that heatSink temp at the point of measurement is generally 20-30 deg C LOWER than temperature 
// of critical components attached to heatsink (due to distance from components to probe)
// use maxHeatSinkT of <60 to ensure <85 deg C temp of components
// this assumes thermistor placement near the heat generating components
// BTW, modest airflow (a single 120mm PC fan) with a large (8x10x3" heatsink should be sufficient for 
// up to 30A output at max power 
#ifndef MCC100A
const byte maxHeatSinkT = 55; // in Centigrades - will start derating here
#else
const byte maxHeatSinkT = 47; // more aggressive derating at high current output
#endif
const byte ABSmaxHeatSinkT = 85; // in Centigrades - will stop the charger altogether here
const byte midHeatSinkT = 45; // turn on the fans here; also wait until cool down to this temp before resuming at the prev power level 
const byte lowHeatSinkT = 35; // turn off the fans here 
//--------------------------------------------------------

const float Aref = 5.0f; // 5V for ATMega328 (Pro Mini), 3.3V for ATSAM (Due) 

//=============== voltage dividers settings ===========================
//--------- some constants for 7520 chips
const float gain_7520 = 5. / 0.512*0.99; // calculate effective gain (Vcc/0.512 per datasheet, plus 1% correction for input resistance)
const float lowerR0_V_7520 = 2.7;

//--------- mains voltage 
// INPUT side constants
#ifdef A7520_mV
// resistor from -5V regulator; should form a ~-.25V divider together with the 
// bottom resistor => >20x * bottom resistor 
// for 2.7k bottom 3resistor, pick between 60k and 82k; 68k is a good choice... 
const float V_o_mV0 = 2.5 - 5.*lowerR0_V_7520 / 68.*gain_7520; // -5V input, 2.7k bottom resistor, 9.76 gain; // ~2.5V for A7520
const float lowerR0_mV = lowerR0_V_7520*gain_7520; // +-0.256V range for input, ~10x gain, 2.7k bottom resistor
const float lowerR_mV = lowerR0_mV;
#else
const float V_o_mV0 = 0;
const float lowerR_mV = 23.79; // 1/(1/27.+1/200.) - in parallel with ISO124 input resistance of 200k
#endif


//--------- battery voltage 
// OUTPUT side constants

#ifdef A7520_V
// resistor from -5V regulator; should form a ~-.25V divider together with the 
// bottom resistor => >20x * bottom resistor 
// for 2.7k bottom resistor, pick between 60k and 82k; 68k is a good choice... 
const float V_o_bV0 = 2.5 - 5.*lowerR0_V_7520 / 68.*gain_7520; // -5V input, 2.7k bottom resistor, 9.76 gain; // ~2.5V for A7520
const float lowerR0_bV = lowerR0_V_7520*gain_7520;   // +-0.256V range for input, Vref/0.512x gain
const float lowerR_bV = lowerR0_bV;
#else
const float V_o_bV0 = 0;
const float lowerR_bV = 23.79; // in parallel with 200k input resistance of the iso124
#endif

//==================================== end voltage dividers setup =========================

//=================================== charger current sensor ==============================

// sensitivity of the sensor
const float k_V_C =
#ifdef OUTC_SENSOR_Tamura_50B
0.03;
#endif
#ifdef OUTC_SENSOR_Tamura_150B
0.01;
#endif
#ifdef OUTC_SENSOR_Tamura_600B
0.0025;
#endif
#ifdef OUTC_SENSOR_Allegro_100U
0.04;
#endif
//=================================== END charger current sensor ==========================

//===================== charger cycle timers =====================================
// stepDelay is a delay between checking for serial commands and printing out the status to Serial line in a serial control mode
// to get the actual delay between such reports, add ~30ms for the actual duration of the data transmission over serial line
// when changing stepDelay, keep (stepDelay+30)*measCycle_len in the 500-1000 range 
// higher stepDelay are advisable in case of non-CHADEMO applications
#ifdef CHADEMO //adds up to 450ms
  const byte stepDelay=30; // primary charger loop delay in milliseconds - should be less than 50 in order to run QC loop properly
  const byte measCycle_len=15; // how many primary loop cycles per display cycle 
#else //adds up to 800ms
  const byte stepDelay=200; // primary charger loop delay in milliseconds 
  const byte measCycle_len=4; // how many primary loop cycles per display cycle 
#endif

const byte AVGCycles = 10; // how many readings of C,V,T (taken every 4ms)  to average for reporting (to CHAdeMO) and display
const byte stopCycles = 5; // how many primary charger cycles to require stop condition to exist before exiting
const byte CV_timeout = 20; // what is the max duration (in secs) CV loop is allowed to spend below C stop; should be > ramp time of charger
//===================== end charger cycle timers =================================

const byte minMains = 30; // min mains voltage to (1) test sensor connectivity and (2) detect mains disconnect 

//PID Configuration
const int32_t pids_Kp_SLOW = 60; // revert to slow PID once the current shows up
const int32_t pids_Kp_FAST = 300; // fast PID to start with
const int32_t pids_Ki = 1; // need small integral term - otherwise we get some constant offset error
const int32_t pids_Kd = 0; // for now, just PI loop

//these are the general system loop states
const uint8_t STATE_TOP_MENU = 0x00;
const uint8_t STATE_CONFIG_PWR = 0x01;
const uint8_t STATE_CONFIG_TIMER = 0x02;
const uint8_t STATE_STARTUP_CFG = 0x03;
const uint8_t STATE_CHARGE_START = 0x04; //charging getting set up
const uint8_t STATE_CHARGE_LOOP = 0x05; //charging is ongoing
const uint8_t STATE_CHARGE_FINISH = 0x06; //quit charging and set everything inert again
const uint8_t STATE_WAIT_TIMEOUT = 0x08;
const uint8_t STATE_SERIALCONTROL = 0x10;
const uint8_t STATE_SETUP_CV = 0x20; //per cell voltage
const uint8_t STATE_SETUP_CELLS = 0x21; //# of cells in pack
const uint8_t STATE_SETUP_CAPACITY = 0x22; //capacity of pack
const uint8_t STATE_SETUP_CALIBRATE = 0x23; // sensitivity calibration only. zero point calibration done automatically on power-on
const uint8_t STATE_FACTORY_RESET = 0xDB; //clear all configuration and restart setup process.
const uint8_t STATE_SHUTDOWN = 0xff;

const uint8_t MSG_THX = 0x00;
const uint8_t MSG_NOBATT = 0x01;
const uint8_t MSG_WRONGPROF = 0x02;
const uint8_t MSG_BMSSTOP = 0x03;
const uint8_t MSG_TIMEOUT = 0x04;
const uint8_t MSG_USRPAUSE = 0x05;
const uint8_t MSG_LOSTIN = 0x06;
const uint8_t MSG_SENSEERROR = 0x07;
const uint8_t MSG_NORMEXIT = 0x08;
const uint8_t MSG_DONE = 0x09;

const uint8_t MSG_LCD_CELLTYPE = 0x00;
const uint8_t MSG_LCD_CV = 0x01;
const uint8_t MSG_LCD_NCELLS = 0x02;
const uint8_t MSG_LCD_CAPACITY = 0x03;
const uint8_t MSG_LCD_CAL0 = 0x04;
const uint8_t MSG_LCD_CAL1 = 0x05;
const uint8_t MSG_LCD_CAL2 = 0x06;
const uint8_t MSG_LCD_CONFIRM = 0x07;
const uint8_t MSG_LCD_PARAMS = 0x08;
const uint8_t MSG_LCD_CFG = 0x09;
const uint8_t MSG_LCD_TOPMENU = 0x0A;
const uint8_t MSG_LCD_INC = 0x0B;
const uint8_t MSG_LCD_OUTC = 0x0C;
const uint8_t MSG_LCD_TOUT = 0x0D;
const uint8_t MSG_LCD_RUN = 0x0E;
const uint8_t MSG_LCD_BLANK = 0x0F;

//namespace StringTable {

	//============================================== define messages ==================
	//-----------------------Navigate Menus--------------------
const uint8_t configMenuLen = 4;
static const char * configMenu[] = { "Run  ", "Pwr  ", "Time ", "RESET"};

#ifndef LCD_SPE
  const char msg_long_0[] PROGMEM = "Thank you for choosing EMotorWerks! BTN to CFG";
  const char msg_short_0[] PROGMEM = "INIT";
  const char msg_long_1[] PROGMEM = "No batt or reverse! ANY BTN to ignore";
  const char msg_short_1[] PROGMEM = "NOBATT";
  const char msg_long_2[] PROGMEM = "Wrong profile!";
  const char msg_short_2[] PROGMEM = "WRONGPROF";
  const char msg_long_3[] PROGMEM = "BMS Stop";
  const char msg_short_3[] PROGMEM = "BMSSTOP";
  const char msg_long_4[] PROGMEM = "Timeout";
  const char msg_short_4[] PROGMEM = "TIMEOUT";
  const char msg_long_5[] PROGMEM = "Paused. RED BTN to exit, GRN to resume";
  const char msg_short_5[] PROGMEM = "USRPAUSE";
  const char msg_long_6[] PROGMEM = "Lost AC";
  const char msg_short_6[] PROGMEM = "LOSTIN";
  const char msg_long_7[] PROGMEM = "Sensor/cal error. Recal/chk wiring";
  const char msg_short_7[] PROGMEM = "SENSEERROR";
  const char msg_long_8[] PROGMEM = "Step complete";
  const char msg_short_8[] PROGMEM = "NORMEXIT";
  const char msg_long_9[] PROGMEM = "Complete! GRN BTN to repeat";
  const char msg_short_9[] PROGMEM = "DONE";
#else 
  const char msg_long_0[] PROGMEM = "Thank you for\nchoosing\nEMotorWerks!\nBTN to CFG";
  const char msg_short_0[] PROGMEM = "INIT";
  const char msg_long_1[] PROGMEM = "No batt or reverse! \nANY BTN to ignore";
  const char msg_short_1[] PROGMEM = "NOBATT";
  const char msg_long_2[] PROGMEM = "Wrong profile!";
  const char msg_short_2[] PROGMEM = "WRONGPROF";
  const char msg_long_3[] PROGMEM = "BMS Stop";
  const char msg_short_3[] PROGMEM = "BMSSTOP";
  const char msg_long_4[] PROGMEM = "Timeout";
  const char msg_short_4[] PROGMEM = "TIMEOUT";
  const char msg_long_5[] PROGMEM = "Paused. RED BTN \nto exit, GRN to \nresume";
  const char msg_short_5[] PROGMEM = "USRPAUSE";
  const char msg_long_6[] PROGMEM = "Lost AC";
  const char msg_short_6[] PROGMEM = "LOSTIN";
  const char msg_long_7[] PROGMEM = "Sensor/cal error. \nRecal/chk wiring";
  const char msg_short_7[] PROGMEM = "SENSEERROR";
  const char msg_long_8[] PROGMEM = "Step complete";
  const char msg_short_8[] PROGMEM = "NORMEXIT";
  const char msg_long_9[] PROGMEM = "Complete! GRN BTN \nto repeat";
  const char msg_short_9[] PROGMEM = "DONE";
#endif

PROGMEM const char* const msg_long_table[] = 	  
{   
  msg_long_0,
  msg_long_1,
  msg_long_2,
  msg_long_3,
  msg_long_4,
  msg_long_5, 
  msg_long_6,
  msg_long_7,
  msg_long_8,
  msg_long_9
};

PROGMEM const char* const msg_short_table[] = 	  
{   
  msg_short_0,
  msg_short_1,
  msg_short_2,
  msg_short_3,
  msg_short_4,
  msg_short_5, 
  msg_short_6,
  msg_short_7,
  msg_short_8,
  msg_short_9
};

#ifndef LCD_SPE
  const char msg_lcd_0[] PROGMEM = "Cell Type:       ";
  const char msg_lcd_1[] PROGMEM = "CV cutoff:       ";
  const char msg_lcd_2[] PROGMEM = "Number of cells: ";
  const char msg_lcd_3[] PROGMEM = "Capacity:        ";
  const char msg_lcd_4[] PROGMEM = "Calibrated zero";
  const char msg_lcd_5[] PROGMEM = "Connect batt. BTN to skip";
  const char msg_lcd_6[] PROGMEM = "Enter actual batt voltage:";
  const char msg_lcd_7[] PROGMEM = "Confirm:      ";
  const char msg_lcd_8[] PROGMEM = "Params      ";
  const char msg_lcd_9[] PROGMEM = "press BTN to adjust";
  const char msg_lcd_10[] PROGMEM = "Action:                   ";
  const char msg_lcd_11[] PROGMEM = "max INput current ";
  const char msg_lcd_12[] PROGMEM = "max OUTput current";
  const char msg_lcd_13[] PROGMEM = "timeout (#min or 0):";
  const char msg_lcd_14[] PROGMEM = "Confirm CHARGE:";
  const char msg_lcd_15[] PROGMEM = "[           ]";
#else
  const char msg_lcd_0[] PROGMEM = "Cell Type:       ";
  const char msg_lcd_1[] PROGMEM = "CV cutoff:       ";
  const char msg_lcd_2[] PROGMEM = "Number of cells: ";
  const char msg_lcd_3[] PROGMEM = "Capacity:        ";
  const char msg_lcd_4[] PROGMEM = "Calibrated zero";
  const char msg_lcd_5[] PROGMEM = "Connect batt. BTN \nto skip";
  const char msg_lcd_6[] PROGMEM = "Enter actual \nbatt voltage:";
  const char msg_lcd_7[] PROGMEM = "Confirm:      ";
  const char msg_lcd_8[] PROGMEM = "Params      ";
  const char msg_lcd_9[] PROGMEM = "press BTN to\nadjust";
  const char msg_lcd_10[] PROGMEM = "Action:                   ";
  const char msg_lcd_11[] PROGMEM = "max INput current ";
  const char msg_lcd_12[] PROGMEM = "max OUTput current";
  const char msg_lcd_13[] PROGMEM = "timeout (#min or 0):";
  const char msg_lcd_14[] PROGMEM = "Confirm CHARGE:";
  const char msg_lcd_15[] PROGMEM = "[           ]";
#endif

PROGMEM const char* const msg_lcd_table[] = 	  
{   
  msg_lcd_0,
  msg_lcd_1,
  msg_lcd_2,
  msg_lcd_3,
  msg_lcd_4,
  msg_lcd_5,
  msg_lcd_6,
  msg_lcd_7,
  msg_lcd_8,
  msg_lcd_9,
  msg_lcd_10,
  msg_lcd_11,
  msg_lcd_12,
  msg_lcd_13,
  msg_lcd_14,
  msg_lcd_15
};

//};
#endif /* CONSTANTS_H_ */
