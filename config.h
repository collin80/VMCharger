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

#endif /* CONFIG_H_ */
