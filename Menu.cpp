#include "Menu.h"

char str[64];

//================== serial comms ========================
// message FROM the charger via serial
void EMWserialMsg(const char *txt) {
  Serial.print("M,");
  Serial.print(txt);
  Serial.println(",E");
}

// message TO the charger via serial
// command syntax: M,ccc,vvv,sss,E
void readSerialCmd(int *cmd_) {
      //+++++++++++++++++++ REWRITE into either async buffer or blocking reads +++++++++++++++++++++ 
  if(Serial.available()>0) {
    if(Serial.read()=='M') {
      // this is a legit command
      Serial.read(); // dispose of comma
      str[0]=Serial.read();
      str[1]=Serial.read();
      str[2]=Serial.read();
      str[3]=0;
      cmd_[0]=atoi(str);
      Serial.read(); // dispose of comma
      str[0]=Serial.read();
      str[1]=Serial.read();
      str[2]=Serial.read();
      str[3]=0;
      cmd_[1]=atoi(str);
      Serial.read(); // dispose of comma
      str[0]=Serial.read();
      str[1]=Serial.read();
      str[2]=Serial.read();
      str[3]=0;
      Serial.read(); // dispose of comma
      if( Serial.read()!='E' || atoi(str)!=getCheckSum(cmd_[0], cmd_[1]) ) {
        cmd_[0]=cmd_[1]=0;
      }
    }
  }
}

int getCheckSum(int val1, int val2) {
  return (val1+val2)%1000;
}

//=========================================== Communication (LCD / Serial) Functions =========================
// main parameter printing function
void printParams(float outV, float outC, int t, float curAH, float maxC, float maxV) {
  if(LCD_on) {
    sprintf(str, "%s - D: %d  ", VerStr, int(milliduty/10000)); 
	myLCD->printStr(0, 0, 2, 0x1f, 0x3f, 0x1f, str);      
    sprintf(str, "I: %dV   ", int(mainsV)); myLCD->printStr(0, 3, 2, 0x1f, 0x3f, 0, str);      
    sprintf(str, "O: %dV, %d.%dA   ", int(outV), int(outC), abs(int(outC*10)%10) ); myLCD->printStr(0, 4, 2, 0x1f, 0x3f, 0, str);      
#ifdef DEBUG0
    sprintf(str, "* t%d-a%d %d %d", targetC_ADC, outC_ADC, int(10000*k_V_C), int(10*V_o_C)); myLCD->printStr(1, 5, 2, 0x1f, 0, 0, str);
#endif
    sprintf(str, "T: %dC ", t); myLCD->printStr(0, 7, 2, 0x1f, 0, 0, str);
    sprintf(str, "%d dAH, %lu sec", int(curAH*10), sec_up); 
	myLCD->printStr(0, 8, 2, 0x1f, 0x3f, 0, str);      
  } else {
    // machine-readable
    // format: [D]uty in 0.1%, [C]urrent in 0.1A, [V]oltage in 1.0V, [T]emp in C, [O]utput AH in 0.1AH, [S]um (checksum)
    sprintf(str, "S:D%03d,C%03d,V%03d,T%03d,O%03d,S%03d", int(milliduty/10000), int(outC*10), int(outV), t, int(curAH*10), getCheckSum(int(outC*10), int(outV)));
    EMWserialMsg(str);
#ifdef DEBUG1
    sprintf(str, "S2:c%03d,v%03d,%05u", int(maxC*10), int(maxV), (unsigned int)millis());
    EMWserialMsg(str);
#endif
  }
}
// printing primitives
void printClrMsg(const byte msg_id, const int del, const byte red, const byte green, const byte blue) {
  if(LCD_on) {
    strcpy_P(str, (char*)pgm_read_word(&(msg_long_table[msg_id]))); 
    myLCD->clrScreen();
  } else {
    strcpy_P(str, (char*)pgm_read_word(&(msg_short_table[msg_id]))); 
  }
  printMsg(str, del, 0, 2, red, green, blue);
}
void printConstStr(int col, int row, int font, byte red, byte green, byte blue, const byte msg_id) {
  strcpy_P(str, (char*)pgm_read_word(&(msg_lcd_table[msg_id]))); 
  printMsg(str, 0, col, row, red, green, blue);
}
void printLabel(const char * label, const byte col, const byte row, const byte red, const byte green, const byte blue) {
  strcpy(str, label);
  printMsg(str, 0, col, row, red, green, blue); 
}
void printMsg(char *str_, const int del, const byte col, const byte row, const byte red, const byte green, const byte blue) {
  if(LCD_on) {
    myLCD->printStr(col, row, 2, red, green, blue, str_);      
    delay(del);
  } else {
    EMWserialMsg(str_);
  }  
}


unsigned int MenuSelector2(byte selection_total, const char * labels[])
{
  byte selection = 0;
  byte temp_selection = 1;
  
  printConstStr(0, 3, 2, 0x1f, 0x3f, 0x1f, MSG_LCD_BLANK);
  printLabel(labels[temp_selection-1], 1, 3, 0x1f, 0x3f, 0x1f);

  while(!selection)
  {
    if(digitalRead(pin_pwrCtrlButton) == HIGH)
    {
      ++temp_selection;
      if(temp_selection > selection_total) temp_selection = 1;
      printConstStr(0, 3, 2, 0x1f, 0x3f, 0x1f, MSG_LCD_BLANK);
      printLabel(labels[temp_selection-1], 1, 3, 0x1f, 0x3f, 0x1f);
      
      // ideally, this should call a StatusDisplay method and simply pass selection index
      // StatusDisplay should encapsulate all the complexities of drawing status info onto the screen
      // alternatively myLCD can be re-purposed for this
    }
    else
    if(digitalRead(pin_pwrCtrl2Button) == HIGH)
    {
      selection = temp_selection;
      printConstStr(0, 3, 2, 0x1f, 0x0, 0x0, MSG_LCD_BLANK);
      printLabel(labels[selection-1], 1, 3, 0x1f, 0x0, 0x0);
      // similar to the above, should delegate display to StatusDisplay object
    } 
    delay(80);
  }

  delay(200);
  
  return selection - 1;
}


byte BtnTimeout(byte n, byte line) {
  while(n > 0) {
    sprintf(str, "%d sec ", n); 
    printMsg(str, 0, 0, line, 0x1f, 0x3f, 0);

    for(byte k=0; k<100; k++) {
      if(digitalRead(pin_pwrCtrlButton)==HIGH || digitalRead(pin_pwrCtrl2Button) == HIGH) return 1;
      delay(10);
    }
    --n;
  }
  return 0;
}

int DecimalDigitInput3(int preset)
{
  byte digit[3] = { preset/100, (preset/10)%10, (preset%10) };
  byte x = 0; // 0-1-2-3-4
  // 0x30 ascii for "0"
  str[1] = 0x0; // eol 

  while(x < 4)
  { 
    if(digitalRead(pin_pwrCtrlButton) == HIGH) {
      if(x > 2) x = 0;
      else {
        // increment digit
        ++digit[x];
        // wrap at 4 (for 100s) or at 9 (for 10s and 1s) 
        if(x == 0 && digit[x] > 4) digit[x] = 0;
        if(digit[x] > 9) digit[x] = 0;
      }      
    } else 
    if(digitalRead(pin_pwrCtrl2Button) == HIGH) {
      ++x;
    } 

    printDigits(0, digit, 1);
  
    if(x < 3) {
      // still on digits. Reprint the digit we are on in a different color now so we see what's being changed
      str[0] = 0x30+digit[x];
      printDigit(x, 0, str);
    } else 
    if(x == 3) {
      // selection made - show all in the 'changing' color
      printDigits(0, digit, 0);
    }
    
    delay(150);
  }
  
  printDigits(8, digit, 0);

  return (digit[0]*100+digit[1]*10+digit[2]);
}

void printDigits(byte start, byte * digit, byte stat) {
  str[0] = 0x30+digit[0];
  printDigit(start++, stat, str);
  str[0] = 0x30+digit[1];
  printDigit(start++, stat, str);
  str[0] = 0x30+digit[2];
  printDigit(start, stat, str);
}
void printDigit(byte x, byte stat, char * str) {
  if(stat==0) printMsg(str, 0, x, 5, 0x1f, 0x3f, 0x0); // yellow
  if(stat==1) printMsg(str, 0, x, 5, 0x8, 0x8, 0x1f); // blue
}
