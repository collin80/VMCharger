#include <Arduino.h>
#include "config.h"

//Strings stored in progmem are special and must be defined in one place and referenced from there

#ifndef LCD_SPE
  prog_char msg_long_0[] PROGMEM = "Thank you for choosing EMotorWerks! BTN to CFG";
  prog_char msg_short_0[] PROGMEM = "INIT";
  prog_char msg_long_1[] PROGMEM = "No batt or reverse! ANY BTN to ignore";
  prog_char msg_short_1[] PROGMEM = "NOBATT";
  prog_char msg_long_2[] PROGMEM = "Wrong profile!";
  prog_char msg_short_2[] PROGMEM = "WRONGPROF";
  prog_char msg_long_3[] PROGMEM = "BMS Stop";
  prog_char msg_short_3[] PROGMEM = "BMSSTOP";
  prog_char msg_long_4[] PROGMEM = "Timeout";
  prog_char msg_short_4[] PROGMEM = "TIMEOUT";
  prog_char msg_long_5[] PROGMEM = "Paused. RED BTN to exit, GRN to resume";
  prog_char msg_short_5[] PROGMEM = "USRPAUSE";
  prog_char msg_long_6[] PROGMEM = "Lost AC";
  prog_char msg_short_6[] PROGMEM = "LOSTIN";
  prog_char msg_long_7[] PROGMEM = "Sensor/cal error. Recal/chk wiring";
  prog_char msg_short_7[] PROGMEM = "SENSEERROR";
  prog_char msg_long_8[] PROGMEM = "Step complete";
  prog_char msg_short_8[] PROGMEM = "NORMEXIT";
  prog_char msg_long_9[] PROGMEM = "Complete! GRN BTN to repeat";
  prog_char msg_short_9[] PROGMEM = "DONE";
#else 
  prog_char msg_long_0[] PROGMEM = "Thank you for\nchoosing\nEMotorWerks!\nBTN to CFG";
  prog_char msg_short_0[] PROGMEM = "INIT";
  prog_char msg_long_1[] PROGMEM = "No batt or reverse! \nANY BTN to ignore";
  prog_char msg_short_1[] PROGMEM = "NOBATT";
  prog_char msg_long_2[] PROGMEM = "Wrong profile!";
  prog_char msg_short_2[] PROGMEM = "WRONGPROF";
  prog_char msg_long_3[] PROGMEM = "BMS Stop";
  prog_char msg_short_3[] PROGMEM = "BMSSTOP";
  prog_char msg_long_4[] PROGMEM = "Timeout";
  prog_char msg_short_4[] PROGMEM = "TIMEOUT";
  prog_char msg_long_5[] PROGMEM = "Paused. RED BTN \nto exit, GRN to \nresume";
  prog_char msg_short_5[] PROGMEM = "USRPAUSE";
  prog_char msg_long_6[] PROGMEM = "Lost AC";
  prog_char msg_short_6[] PROGMEM = "LOSTIN";
  prog_char msg_long_7[] PROGMEM = "Sensor/cal error. \nRecal/chk wiring";
  prog_char msg_short_7[] PROGMEM = "SENSEERROR";
  prog_char msg_long_8[] PROGMEM = "Step complete";
  prog_char msg_short_8[] PROGMEM = "NORMEXIT";
  prog_char msg_long_9[] PROGMEM = "Complete! GRN BTN \nto repeat";
  prog_char msg_short_9[] PROGMEM = "DONE";
#endif

PROGMEM const char *msg_long_table[] = 	  
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

PROGMEM const char *msg_short_table[] = 	  
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
  prog_char msg_lcd_0[] PROGMEM = "Cell Type:       ";
  prog_char msg_lcd_1[] PROGMEM = "CV cutoff:       ";
  prog_char msg_lcd_2[] PROGMEM = "Number of cells: ";
  prog_char msg_lcd_3[] PROGMEM = "Capacity:        ";
  prog_char msg_lcd_4[] PROGMEM = "Calibrated zero";
  prog_char msg_lcd_5[] PROGMEM = "Connect batt. BTN to skip";
  prog_char msg_lcd_6[] PROGMEM = "Enter actual batt voltage:";
  prog_char msg_lcd_7[] PROGMEM = "Confirm:      ";
  prog_char msg_lcd_8[] PROGMEM = "Params      ";
  prog_char msg_lcd_9[] PROGMEM = "press BTN to adjust";
  prog_char msg_lcd_10[] PROGMEM = "Action:                   ";
  prog_char msg_lcd_11[] PROGMEM = "max INput current ";
  prog_char msg_lcd_12[] PROGMEM = "max OUTput current";
  prog_char msg_lcd_13[] PROGMEM = "timeout (#min or 0):";
  prog_char msg_lcd_14[] PROGMEM = "Confirm CHARGE:";
  prog_char msg_lcd_15[] PROGMEM = "[           ]";
#else
  prog_char msg_lcd_0[] PROGMEM = "Cell Type:       ";
  prog_char msg_lcd_1[] PROGMEM = "CV cutoff:       ";
  prog_char msg_lcd_2[] PROGMEM = "Number of cells: ";
  prog_char msg_lcd_3[] PROGMEM = "Capacity:        ";
  prog_char msg_lcd_4[] PROGMEM = "Calibrated zero";
  prog_char msg_lcd_5[] PROGMEM = "Connect batt. BTN \nto skip";
  prog_char msg_lcd_6[] PROGMEM = "Enter actual \nbatt voltage:";
  prog_char msg_lcd_7[] PROGMEM = "Confirm:      ";
  prog_char msg_lcd_8[] PROGMEM = "Params      ";
  prog_char msg_lcd_9[] PROGMEM = "press BTN to\nadjust";
  prog_char msg_lcd_10[] PROGMEM = "Action:                   ";
  prog_char msg_lcd_11[] PROGMEM = "max INput current ";
  prog_char msg_lcd_12[] PROGMEM = "max OUTput current";
  prog_char msg_lcd_13[] PROGMEM = "timeout (#min or 0):";
  prog_char msg_lcd_14[] PROGMEM = "Confirm CHARGE:";
  prog_char msg_lcd_15[] PROGMEM = "[           ]";
#endif

PROGMEM const char *msg_lcd_table[] = 	  
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
