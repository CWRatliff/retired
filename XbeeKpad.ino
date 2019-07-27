//190621 - added compass command
//190701 - keypad initial code
//190714 - debounce for ZERO, STOP, COMPASS
//190726 - keypad instead of xbox

#include <Keypad.h>

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {49, 47, 45, 43}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {48, 46, 44, 42}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"
//#include <LiquidCrystal.h>

#define FALSE       0
#define TRUE        (!FALSE)
#define BUFF        256

Adafruit_LiquidCrystal lcd(0);

//  Xbee  Mega
//  2     19 rx1
//  3     18 tx1

#define DIRECTION   'D'     // one digit basic tandby commands
#define EXECUTE     'E'     // execute autopilot maneover
#define FUNCTION    'F'     // multi digit input

int   error = 0;
int   speed;
char  inpt;

int   exeflag = FALSE;
int   lbflag = FALSE;

char  ibuffer[BUFF];              // Xbee software input buffer
int   itail = 0;
int   ihead = 0;
int   piflag = FALSE;

int   epoch;                      // last xmission time
//================================================================
// 1-byte int to two hex bytes
void btoh(int binp, int &hob, int &lob) {
  hob = binp / 16;
  lob = binp - hob * 16;
  }
//===============================================================
// print text numbers to lcd
char* lcdout(char *s) {
  char  c;
  while (isDigit(*s) || *s == '-') {
    c = *s++;
    lcd.print(c);
    }
  return (s);
  }
//===============================================================
// cvt hex digits to uint
int shtoi(char* st) {
  char  c;
  int   hval = 0;

  for (c = *st; isHexadecimalDigit(c); c = *++st) {
    if (c >= 'a')
      c -= ('a' - 'A');
    c = c - '0';
    if (c > 9)
      c -= 7;
    hval = hval * 16 + c;
    }
  return (hval);
  }
//===================================================================  
void xmit(char code) {
  Serial1.print("{");
  Serial1.print(code);
  Serial1.print("}");
  epoch = millis();
  } 
//===================================================================  
void xmithex(char code, int amt) {
  Serial1.print("{");
  Serial1.print(code);
  Serial1.print(lowByte(amt), HEX);
  Serial1.print("}");
  epoch = millis();
  } 
//===================================================================  
void xmitnum(char code, char num) {
  Serial1.print("{");
  Serial1.print(code);
  Serial1.print(num);
  Serial1.print("}");
  epoch = millis();
  } 
//====================================================================
void setup(){
 
  Serial.begin(115200);

  lcd.begin(20, 4);
  lcd.clear();
  Serial1.begin(9600);        // Xbee on hardware port

    lcd.setCursor(0, 0);
    lcd.print("Spd: ");
    lcd.setCursor(0, 1);
    lcd.print("Str: ");
    lcd.setCursor(0, 2);
    lcd.print("Hdg: ");

//   xmit('*');             // ping, were alive
}
//=========================================================================
void loop() {
  /* You must Read Gamepad to get new values and set vibration values
     ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
     if you don't enable the rumble, use ps2x.read_gamepad(); with no values
     You should call this at least once a second
   */  
  char* str;
  int   spd;
  int   ang;


   //-----------------------------------------Xbee input from bot
   while (Serial1.available()) {
    inpt = Serial1.read();
    if (inpt == '{') {
      ihead = 0;
      continue;
      }
    if (inpt == '}') {
      ibuffer[ihead] = '\0';
      piflag = TRUE;
      break;
      }
    ibuffer[ihead++] = inpt;
    }
  if (piflag) {
    Serial.println(ibuffer);
    str = &ibuffer[1];

    if (ibuffer[0] == 'v') {
      lcd.setCursor(5, 0);
      lcdout(str);                        // print speed
      lcd.print("   ");
      }
    if (ibuffer[0] == 's') {
      lcd.setCursor(5, 1);
      lcdout(str);                        // print steering
      lcd.print("   ");
      }
    if (ibuffer[0] == 'h') {
      lcd.setCursor(5, 2);
      lcdout(str);                        // print heading
      lcd.print("   ");
      }
    if (ibuffer[0] == 'r') {
      lcd.setCursor(5, 3);
      lcdout(str);                        // print roll
      lcd.print("   ");
      }
    if (ibuffer[0] == 'p') {
      lcd.setCursor(10, 3);
      lcdout(str);                        // print pitch
      lcd.print("   ");
      }
    piflag = FALSE;
    }

    //-----------------------------------------------keypad inputs
    char kpad = keypad.getKey();
    if (kpad) {
      Serial.print(kpad);


      if (kpad == '*') {
        exeflag = TRUE;
        lbflag = FALSE;
        }
        
      else if (kpad == '#') {
        lbflag = TRUE;
        exeflag = FALSE;
        }
        
      else if (exeflag && kpad >= '0' && kpad <= '9') {
        xmitnum(EXECUTE, kpad);
        exeflag = FALSE;
        }
        
      else if (lbflag && kpad >= '0' && kpad <= '9')
        xmitnum(FUNCTION, kpad);
        lbflag = FALSE;
        }

      else if (kpad >= '0' && kpad <= '9') {
        xmitnum(DIRECTION, kpad);
        }

//    if ((millis() - epoch) > 1000) {}
//      xmit('*');                          // were still alive
//    epoch = millis();
      
  delay(50);  
  }
