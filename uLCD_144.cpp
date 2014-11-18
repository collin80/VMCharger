#include "uLCD_144.h"

//==================================== SCREEN FUNCTION LIBRARY ========================
//-------------- define all function members ----------    
uLCD_144::uLCD_144(int baud) {
  Serial.begin(baud);
  // display requires some time to init before sending auto-baud command
  delay(1000);
  Serial.write(0x55);
  isAlive_ = waitAck();
}

// process color data
byte uLCD_144::getMSB(byte red, byte green, byte blue) {
  return red*8 + green/8;
}
byte uLCD_144::getLSB(byte red, byte green, byte blue) {
  return (green & 0x7)*32 + blue;
}
int uLCD_144::waitAck() {
  int x=0;  
  int z=0;
  do {
    x=Serial.read();
    delay(5);
    z++;
  } while(x!=0x06 && z<100); // wait for screen acknowledgement or 100 cycles
  
  if(x==0x06) return 1; // LCD is there
  // LCD not there
  return 0;
}

int uLCD_144::isAlive() {
  return isAlive_;
}

void uLCD_144::setContrast(int contrast) {
  // range from 0 to 0x0f
  Serial.write(0x59);
  Serial.write(0x02);
  Serial.write(contrast);
  waitAck();  
}

void uLCD_144::clrScreen() {
  Serial.write(0x45);
  waitAck();
}  
  
void uLCD_144::setBgColor(byte red, byte green, byte blue) {
  Serial.write(0x42); //set screen color
  Serial.write(getMSB(red, green, blue));
  Serial.write(getLSB(red, green, blue));
  waitAck();
}

void uLCD_144::setPenSize(int size) {
  // 0 = solid, 1 = wire
  Serial.write(0x70);
  Serial.write(size);
  waitAck();
}

void uLCD_144::setFont(int font) {
  // 0 = small, 1 = med, 2 = large
  Serial.write(0x46);
  Serial.write(font);
  waitAck();
}

void uLCD_144::setOpacity(int opacity) {
  // 0 = transparent, 1 = opaque
  Serial.write(0x4f);
  Serial.write(opacity);
  waitAck();
}

// draw pixel
void uLCD_144::drawPixel(int x, int y, byte red, byte green, byte blue) {
  Serial.write(0x50);
  Serial.write(x);
  Serial.write(y);
  Serial.write(getMSB(red, green, blue));
  Serial.write(getLSB(red, green, blue));
  waitAck();
}

// draw circle
void uLCD_144::drawCircle(int x, int y, int r, byte red, byte green, byte blue) {
  Serial.write(0x43);
  Serial.write(x);
  Serial.write(y);
  Serial.write(r);
  Serial.write(getMSB(red, green, blue));
  Serial.write(getLSB(red, green, blue));
  waitAck();
}

void uLCD_144::printStr(int col, int row, int font, byte red, byte green, byte blue, const char *str) {
  Serial.write(0x73);
  Serial.write(col);
  Serial.write(row);
  Serial.write(font);
  Serial.write(getMSB(red, green, blue));
  Serial.write(getLSB(red, green, blue));
  Serial.print(str);
  Serial.write((uint8_t)0);
  waitAck();
}

void uLCD_144::printGRStr(int x, int y, int font, byte red, byte green, byte blue, int w, int h, const char *str) {

  Serial.write(0x53);
  Serial.write(x);
  Serial.write(y);
  Serial.write(font);
  Serial.write(getMSB(red, green, blue));
  Serial.write(getLSB(red, green, blue));
  Serial.write(w);
  Serial.write(h);
  Serial.print(str);
  Serial.write((uint8_t)0);
  waitAck();
}

void uLCD_144::drawLine(int  x1, int y1, int x2, int y2, byte red, byte green, byte blue) {
  Serial.write(0x4C);
  Serial.write(x1);
  Serial.write(y1);
  Serial.write(x2);
  Serial.write(y2);
  Serial.write(getMSB(red, green, blue));
  Serial.write(getLSB(red, green, blue));
  waitAck();
}

void uLCD_144::drawRect(int x1, int y1, int x2, int y2, byte red, byte green, byte blue) {
  Serial.write(0x72);
  Serial.write(x1);
  Serial.write(y1);
  Serial.write(x2);
  Serial.write(y2);
  Serial.write(getMSB(red, green, blue));
  Serial.write(getLSB(red, green, blue));
  waitAck(); 
}
//================================ END SCREEN FUNCTION LIBRARY ========================
