#ifndef MENU_H
#define MENU_H

#include <Arduino.h>
#include "config.h"

#ifdef LCD_SPE
#include "uLCD_144_SPE.h"
extern uLCD_144_SPE *myLCD;
#else
#include "uLCD_144.h"
extern uLCD_144 *myLCD;
#endif

void printParams(float outV, float outC, int t, float curAH, float maxC, float maxV);
void printClrMsg(const uint8_t msg_id, const int del, const uint8_t red, const uint8_t green, const uint8_t blue);
void printConstStr(int col, int row, int font, uint8_t red, uint8_t green, uint8_t blue, const uint8_t msg_id);
void printLabel(const char * label, const uint8_t col, const uint8_t row, const uint8_t red, const uint8_t green, const uint8_t blue);
void printMsg(char *str_, const int del, const uint8_t col, const uint8_t row, const uint8_t red, const uint8_t green, const uint8_t blue);
unsigned int MenuSelector2(uint8_t selection_total, const char * labels[]);
uint8_t BtnTimeout(uint8_t n, uint8_t line);
int DecimalDigitInput3(int preset);
void printDigits(uint8_t start, uint8_t * digit, uint8_t stat);
void printDigit(uint8_t x, uint8_t stat, char * str);
void EMWserialMsg(const char *txt);
void readSerialCmd(int *cmd_);
int getCheckSum(int val1, int val2);
bool getSerialCmd();
void pollSerial();
bool processSerial();
void dotsDelay(const int del, byte ndots, const byte row);

#endif