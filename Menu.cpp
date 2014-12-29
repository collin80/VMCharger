#include "Menu.h"
#include "avr/pgmspace.h"
#include "config.h"
#include "constants.h"
#include "globals.h"
#include "ConfigStruct.h"
#include "ValueTranslators.h"
#include "buttons.h"

char str[64];
char SerialStr[SerialStrSize+2]; // buffer for the serial command
char SerialCommand[SerialStrSize+2]; // this is where the command will actually be stored

//================== serial comms ========================
// message FROM the charger via serial
void EMWserialMsg(const char *txt) {
  Serial.print("M,");
  Serial.print(txt);
  Serial.println(",E");
}

bool getSerialCmd() 
{
	// serial control of the charger is enabled
    configuration.mainsC=300; // remove limit on input side?
    sprintf(str, "R:M%03d,V%03d,c%03d,v%03d", int(mainsV), int(outV), int(configuration.CC), int(maxOutV));
    EMWserialMsg(str); // send 'ready' status - expect controller to respond within 200ms

    // listen to commands - format 'M,ccc,vvv,sss,E' where sss is a checksum
    cmd[0]=cmd[1]=0; // reset all
    readSerialCmd(cmd);
          
    if(cmd[0]>0 && cmd[1]>0) {
        // echo reception
        sprintf(str, "E:%d,%d", cmd[0], cmd[1]);
        sprintf(SerialCommand, "M,%03d,%03d,%03d,E", cmd[0], cmd[1], getCheckSum(cmd[0], cmd[1]) );
        EMWserialMsg(str);

        if(cmd[0]>=500) {
			POWER_DIRECTION=-1;
			digitalWrite(pin_dirCtrl, HIGH);
			cmd[0]-=500; // remove offset
		} else {
			POWER_DIRECTION=1;
			digitalWrite(pin_dirCtrl, LOW);
		}
		delay(100);

        configuration.CC=cmd[0];
        maxOutV=cmd[1];     
        // move to charge stat
        return true;
     } 
	return false;
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

void pollSerial() 
{
	static byte si=0; // serial command byte counter
	while(Serial.available()) {
        str[0]=Serial.read();
        if(str[0]=='M') si=0; // reset to the beginning of command
        if(si<SerialStrSize-1) {
          SerialStr[si++]=str[0];
        } else {
          SerialStr[SerialStrSize-1]=0; // this is supposed to be the end of the command
          if(str[0]=='E') strcpy(SerialCommand, SerialStr);
          si=0;
        }
    }
}

bool processSerial() 
{
	cmd[0]=cmd[1]=0; // reset all
    str[0]=SerialCommand[2]; // skipping 'M,'
    str[1]=SerialCommand[3];
    str[2]=SerialCommand[4];
    str[3]=0;
    cmd[0]=atoi(str);
    str[0]=SerialCommand[6]; // skipping ','
    str[1]=SerialCommand[7];
    str[2]=SerialCommand[8];
    str[3]=0;
    cmd[1]=atoi(str);
    str[0]=SerialCommand[10]; // skipping ','
    str[1]=SerialCommand[11];
    str[2]=SerialCommand[12];
    str[3]=0;
    if(atoi(str)!=getCheckSum(cmd[0], cmd[1])) { // checksum did not check out
		cmd[0]=0;
        cmd[1]=0;
    }
    if(cmd[0]>1 && cmd[1]>1) {
		// valid output power command
		// voltage
		maxOutV=cmd[1];
		// current
		// FOR PROPERLY CONFIGURED QC-25 ONLY: 0-499 denotes positive flow (normally buck), 500-999 - negative (boost)
		if(cmd[0]>=500) {
			if(POWER_DIRECTION==1) {
				// change in direction, stop PWM
				PWM_enable_=0;
				state = STATE_CHARGE_FINISH;
				return -1; // halt
			}
			POWER_DIRECTION=-1;
			cmd[0]-=500; // remove offset
		} else {
			if(POWER_DIRECTION==-1) {
				// change in direction, stop PWM
				PWM_enable_=0;
				state = STATE_CHARGE_FINISH;
				return -1;
			}
			POWER_DIRECTION=1;
		}
		// set the current to the requested value BUT ONLY if we are not exceeding maxOutV
		// if maxOutV exceeded, the charger takes control and starts reducing the current
		if(!CVreached) maxOutC=maxOutC1=getAllowedC(cmd[0]); // this also allows for temp derating (in getAllowedC)
		state = STATE_CHARGE_START; //just to be sure we're still set up properly. START will automatically transition back to CHARGE_LOOP
		PWM_enable_=1; // re-enable if needed
    } else {
		// could be a special command
        // 'M,001,000,001,E' is STOP 
        if(cmd[0]==1 && cmd[1]==0) {
			PWM_enable_=0;
			state = STATE_CHARGE_FINISH;
            return false; // full stop
        }
    }
    // send status now - this is ~45 symbols. At 19200 bps, this is  ~30ms
    printParams(outV, outC, normT, AH_charger, maxOutC1, maxOutV);
	return true;
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
    if(isButton1Down())
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
    if(isButton2Down())
    {
      selection = temp_selection;
      printConstStr(0, 3, 2, 0x1f, 0x0, 0x0, MSG_LCD_BLANK);
      printLabel(labels[selection-1], 1, 3, 0x1f, 0x0, 0x0);
      // similar to the above, should delegate display to StatusDisplay object
    } 
    delay(125);
	clearButtons();
  }

  delay(250);
  clearButtons();
  
  return selection - 1;
}


byte BtnTimeout(byte n, byte line) {
  while(n > 0) {
    sprintf(str, "%d sec ", n); 
    printMsg(str, 0, 0, line, 0x1f, 0x3f, 0);

    for(byte k=0; k<10; k++) {
      if(isButton1Down() || isButton2Down()) return 1;
      delay(100);
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
    if(isButton1Down()) {
      if(x > 2) x = 0;
      else {
        // increment digit
        ++digit[x];
        // wrap at 4 (for 100s) or at 9 (for 10s and 1s) 
        if(x == 0 && digit[x] > 4) digit[x] = 0;
        if(digit[x] > 9) digit[x] = 0;
      }      
    } else 
    if(isButton2Down()) {
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
    
    delay(200);
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

// this prints running dots to keep user entertained during delays
void dotsDelay(const int del, byte ndots, const byte row) {
  for(; ndots>0; ndots--) {
    if(LCD_on) {
      myLCD->printStr(ndots, row, 2, 0, 0x1F, 0, ".");      
      delay(del);
    } else {
      EMWserialMsg(".");
    }      
  }
}