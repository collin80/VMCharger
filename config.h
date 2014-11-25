/*
 * config.h
 *
 * allows the user to configure
 * static parameters.
 *
 * Note: Make sure with all pin defintions of your hardware that each pin number is
 *       only defined once.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

//----------- DEBUG switch - careful, this disables many safety features...---------------
// #define DEBUG0 // ad-hoc stuff - targetC at the moment
// #define DEBUG1 // additional printouts / delays / etc.
// #define DEBUG2 // even more printouts etc
//----------------------------------------------------------------------------------------

//============================== MAIN SWITCHES FOR EMotorWerks chargers =================
#define VerStr "V16"
#define SmartCharge // 12kW chargers
#define PFC // is this a PFC unit? no need to have this when QuickCharge is selected

//#define QuickCharge // 25kW chargers - ONLY FOR AC

//#define DCDC // module is used in DC-DC mode (buck or boost)
//#define DCDC_BUCK // the voltage readings are swapped

// #define SLOWPID // for universal voltage non-CHAdeMO units, slow down PID; may also be needed for any PFCDirect units due to time constant of the PFC circuit
// ===================================================================================================

// ================= SUB-SWITCHES DEFAULT VALUES FOR EMotorWerks chargers ========================
// by-version defaults
#ifdef SmartCharge // the latest version of the PFC version of SmartCharge-12000 12kW chargers
#define OUTC_SENSOR_Allegro_100U // default is Allegro_100U
#define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124)
#define PC817 // mains voltage sensing based on a crude regular opto (V13 boards)
#endif
#ifdef QuickCharge // the latest version of the AC-FED PFCDirect or QuickCharge-25000 25kW chargers
#define OUTC_SENSOR_Tamura_150B // Tamura_150B is default for PFCdirect 25kW units only V3 (after Dec 2 2013), Tamura_50B for high-voltage units (800V)
#define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124)
#define A7520_mV // using A7520 optoisolation for mV sensing? (as opposed to ISO124)
#define PFC // all QuickCharge units are PFC
#define PFCdirect // is this a PFCDirect 25kW unit?
#define NEG_CSENSE // in post-Oct'13 PFCdirect units, current runs in opposite direction to make 3.3V logic compatible
#endif
#ifdef DCDC
//  #define OUTC_SENSOR_Tamura_600B // Tamura_600B - this is for low-voltage DC-DC only
#define OUTC_SENSOR_Tamura_150B // Tamura_150B is default for high-voltage DC-DC
#define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124)
#define A7520_mV // using A7520 optoisolation for mV sensing? (as opposed to ISO124)
#define DCinput // is this being connected to the DC input or AC? matters for input voltage sense 
#define PFCdirect
// #define LOWVOLTAGE // low voltage = lower PWM freq

// in all DCDC units, current runs in opposite direction (for BUCK units, this means that we run the output
// wire through the same sensor as normally used on the high side)
#define NEG_CSENSE 

// #define INC_ASOUT // in early DCDC_BUCK units, input sensor is used as output sensor; also, need to spec positive current direction in this case
#endif

// universal defines
#define LCD_SPE // are we using the SPE version of the LCD (shipped after October 2013)
#ifndef DCDC
// #define drop110power // reduce power to ~1.5kW when connected to 110VAC?
// #define CHECKMAINS
#endif
// Zero motorcycles special - run charger for DeltaQ signal - this assumes D6 is pulled up by 5k to 3.3v and is 
// connected to Zero's deltaQ line on a battery or bike. Test by pulling D6 to ground with 2.2k resistor
// #define DELTAQ  
// ===================================================================================================


//------------------------------ MAIN SWITCHES STORAGE AREA - OVERRIDES ONLY! --------------------
// #define MCC100A // 100A output rating - use ONLY with a custom-wound inductor
// #define A7520_V // using A7520 optoisolation for outV sensing? (as opposed to ISO124 chip used in earlier versions)
// #define A7520_mV // using A7520 optoisolation for mV sensing? (as opposed to ISO124 chip used in earlier versions)
// #define PC817 // mains voltage sensing based on a crude regular opto (V13 boards)
// #define PFC // is this a PFC unit?
// #define PFCdirect // is this a PFCDirect 25kW unit?
// #define UVLO // enable gate supply undervoltage protection?
// #define IND_Temp // do we have a second temp probe on the inductor?
//#define debugpower // doubles the power limits - careful - it may blow up your charger
//------------------------------- END MAIN SWITCHES STORAGE AREA ------------------------------------

#define serialspeed 19200 // 115 is unstable, 38.4 is a bit unstable...
#define SerialStrSize 15 // M,ccc,vvv,sss,E

// sensor constants - moved them to here as they are changed very often depending on what unit we have
// 1M in low-voltage units (up to 150V)
// 2M in SmartCharge-12000 standard units
// 2.4M in most PFCdirect units
// 3M in newer PFCdirect units
// 5-6M in high voltage units (but usually only on high-side)
// However, the value here is 1/1000 of the actual resistance you used 2M = 2000, 3M = 3000, etc.
const float upperR0_mV = 2000.;
const float upperR0_bV = 2000.;

// from Oct 10 2013, this defaults to 20kHz due to use of faster IGBTs. For kits with older IGBTs, use 60-70
// for 60hz line frequency, period has to ideally be 260 / N, where N is an integer
// for 50hz line frequency, 312 / N
#ifdef LOWVOLTAGE
const int period = 130; // 8khz for low-voltage, high-current applications; 60hz line freq
#else 
//   const int period=52; // 52us (~20khz) for normal-voltage applications; 60hz line freq
  const int period=65; // 65us (~16khz) for high-voltage applications; 60hz line freq
//  const int period=86; // 86us (~12kHz) for high-voltage applications with high inductance or low inductor voltage 
//const int period = 130; // 130us (~8kHz) for medium-voltage applications with high inductance or low inductor voltage 
#endif

#endif /* CONFIG_H_ */
