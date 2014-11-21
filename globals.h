#include "config.h"
//Evil global variables shared by various code sections. Refactor to classes and remove these things.
extern uint8_t LCD_on;
extern int32_t milliduty;
extern float mainsV;
extern uint32_t sec_up;


extern uint8_t breakCnt;
extern uint8_t breakCycle;

extern uint32_t timer;
extern uint32_t timer_ch;
extern uint32_t timer_comm;
extern uint32_t timer_irq;
extern uint32_t deltat;

extern float outV;
extern float outC;
extern float AH_charger;

extern uint8_t charger_run;
extern uint8_t state;
extern uint8_t normT;

extern int32_t timeOut;
extern float maxOutV;
extern float maxMainsC;
extern float maxOutC = 0.;
extern float maxOutC1 = 0;

extern uint8_t PWM_enable_;

extern int cmd[2];

#ifdef LCD_SPE
  #include "uLCD_144_SPE.h"
  extern uLCD_144_SPE *myLCD;
#else
  #include "uLCD_144.h"
  extern uLCD_144 *myLCD;
#endif