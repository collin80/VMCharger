/*  

Library:     pwm01.h (version 01)
Date:        2/11/2013
Written By:  randomvibe  

Purpose:     
   Setup one or two unique PWM frequenices directly in sketch program,
   set PWM duty cycle, and stop PWM function.

User Functions:     
   pwm_set_resolution(int res) ~  setup PWM resolution; up to 16 bit
   pwm_setup( uint32_t pwm_pin, uint32_t pwm_freq, int iclock) ~ Setup PWM on clock-A (iclock=1) or clock-B (iclock-2)
   pwm_write_duty( uint32_t pwm_pin, uint32_t pwm_duty) ~ Write PWM duty cycle
   pwm_stop( uint32_t pwm_pin) ~ Force PWM stop

Notes:
   - Applies to Arduino-Due board, PWM pins 6, 7, 8 & 9.
   - Libary Does not operate on the TIO pins.
   - Unique frequencies set via PWM Clock-A ("CLKA") and Clock-B ("CLKB")
     Therefore, up to two unique frequencies allowed.
   - Set max duty cycle counts (pwm_max_duty_Ncount) equal to 255
     per Arduino approach.  This value is best SUITED for low frequency
     applications (2hz to 40,000hz) such as PWM motor drivers, 
     38khz infrared transmitters, etc.
   - Future library versions will address high frequency applications.
   - Arduino's "wiring_analog.c" function was very helpful in this effort.
      
*/

#include "Arduino.h"


static int       pwm_resolution_nbit = 8;
static uint32_t  pwm_clockA_freq = 0;
static uint32_t  pwm_clockB_freq = 0;
static uint32_t  pwm_max_duty_Ncount = 255;


void pwm_set_resolution(int res)
{
	pwm_resolution_nbit = res;
}


void pwm_set_clockA_freq(uint32_t  val)
{
	pwm_clockA_freq = val;
}


void pwm_set_clockB_freq(uint32_t  val)
{
	pwm_clockB_freq = val;
}


static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) 
{
    if (from == to)
        return value;
    if (from > to)
        return value >> (from-to);
    else
        return value << (to-from);
}



// MAIN PWM INITIALIZATION
//--------------------------------
void  pwm_setup( uint32_t  pwm_pin,  uint32_t  pwm_freq,  int iclock  ) 
{
    uint32_t  pwm_duty = 0;    
    uint32_t  chan = g_APinDescription[pwm_pin].ulPWMChannel; 
      
    // SET CLOCK FREQUENCY
    //------------------------
    if (iclock==1)  pwm_set_clockA_freq( pwm_max_duty_Ncount*pwm_freq ); 
    if (iclock==2)  pwm_set_clockB_freq( pwm_max_duty_Ncount*pwm_freq ); 


    if (pwm_pin>=6 && pwm_pin<=9)
    {    
        // SET PWM RESOLUTION
        //------------------------
        pwm_duty = mapResolution( pwm_duty, pwm_resolution_nbit, PWM_RESOLUTION);
    
        // PWM STARTUP AND SETUP CLOCK
        //-------------------------------
        pmc_enable_periph_clk( PWM_INTERFACE_ID );
        PWMC_ConfigureClocks( pwm_clockA_freq, pwm_clockB_freq, VARIANT_MCK );   
     
        // SETUP PWM FOR pwm_pin
        //------------------------
        PIO_Configure( g_APinDescription[pwm_pin].pPort,  g_APinDescription[pwm_pin].ulPinType,  g_APinDescription[pwm_pin].ulPin,  g_APinDescription[pwm_pin].ulPinConfiguration);
        if (iclock==1)  PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
        if (iclock==2)  PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKB, 0, 0);
        PWMC_SetPeriod(PWM_INTERFACE, chan, pwm_max_duty_Ncount);
        PWMC_SetDutyCycle(PWM_INTERFACE, chan, pwm_duty);
        PWMC_EnableChannel(PWM_INTERFACE, chan);
    }

}



// WRITE DUTY CYCLE
//--------------------------------
void  pwm_write_duty( uint32_t  pwm_pin,  uint32_t  pwm_duty ) 
{
    if (pwm_pin>=6 && pwm_pin<=9)
    {    
        pwm_duty = mapResolution( pwm_duty, pwm_resolution_nbit, PWM_RESOLUTION);
        
        uint32_t  chan = g_APinDescription[pwm_pin].ulPWMChannel;
       
        PWMC_SetDutyCycle(PWM_INTERFACE, chan, pwm_duty);
    }
};



// FORCE PWM STOP
//--------------------------------
void  pwm_stop( uint32_t  pwm_pin ) 
{
    pinMode(pwm_pin, OUTPUT);      // sets the digital pin as output
    digitalWrite(pwm_pin, LOW);    // sets the LED off
};

