/*
Implement interrupt driven button code. This automatically debounces as the interrupt only sets the bit, 
clearing the button requires reading its state. These routines are dead simple but allow for 
button presses to be registered almost no matter what.
*/

#include "buttons.h"

volatile boolean button1Down;
volatile boolean button2Down;

void button1Int()
{
	button1Down = true;
}

void button2Int()
{
	button2Down = true;
}

void setupButtons()
{
	//for the Due you'd use these two lines
	//attachInterrupt(pin_pwrCtrlButton, button1Int, RISING);
	//attachInterrupt(pin_pwrCtrl2Button, button2Int, RISING);

	//But, the ATMEGA chip has only two interrupts and they are 0 and 1 so use the below instead
	//luckily, the two interrupts on the ATMEGA just happen to be on the two button pins
	attachInterrupt(0, button1Int, RISING);
	attachInterrupt(1, button2Int, RISING);

	clearButtons();
}

boolean isButton1Down()
{
	boolean tmp = button1Down;
	button1Down = false;
	return tmp;
}

boolean isButton2Down()
{
	boolean tmp = button2Down;
	button2Down = false;
	return tmp;
}

//set the buttons as unpressed
void clearButtons()
{
	button1Down = false;
	button2Down = false;
}