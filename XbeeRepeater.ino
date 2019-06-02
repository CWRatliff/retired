// 190214

#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"

Adafruit_LiquidCrystal lcd(0);
//  Xbee  Mega
//  2     19 rx1
//  3     18 tx1

char  ibuffer[50];        // xbee input buffer
int   irow = 0;
int   bndx;

//====================================================================
void setup() {

  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Hello, World");
  lcd.setCursor(0, 1);
  lcd.print("from 350 VdM");
  lcd.setCursor(0, 2);
  lcd.print("Row 3");
  lcd.setCursor(0, 3);
  lcd.print("Row 4");
  lcd.setCursor(0, 0);
  Serial1.begin(9600);
  delay(500); 
  bndx = 0;
  ibuffer[0] = '\0';
  }
//====================================================================
void loop() {
  char  cin;
  
  if (Serial1.available()) {
    cin = Serial1.read();
    ibuffer[bndx++] = cin;
    ibuffer[bndx] = '\0';
    if (cin == '}') {
      if (ibuffer[0] == '{' && ibuffer[1] != '*') {
        lcd.setCursor(0, irow++);
        lcd.print(ibuffer);
        lcd.print("        ");
        irow %= 4;
        }
      bndx = 0;           // reset input buffer
      ibuffer[0] = '\0';
      }
    }
  }



