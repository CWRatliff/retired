//190621 - added compass command
// joystick algorithm adapted from Calvin Hass http://www.impulseadventure.com/elec/

#include <Keypad.h>

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {51, 49, 47, 45}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {50, 48, 46, 44}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

#include <PS2X_lib.h>
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"
//#include <LiquidCrystal.h>

#define FALSE       0
#define TRUE        (!FALSE)
#define BUFF        256

#define PS2_DAT        11
#define PS2_CMD        10
#define PS2_SEL         9
#define PS2_CLK         8

Adafruit_LiquidCrystal lcd(0);

//  Xbee  Mega
//  2     19 rx1
//  3     18 tx1

#define ACK         '*'     // Ping
#define ANGLE       'A'     // tool boom angle
#define BACK        'B'     // reverse drive motion
#define COMPASS     'C'     // steer to current compass hdg
#define DIRECTION   'D'     // tool base
#define EXECUTE     'E'     // execute autopilot maneover
#define FWD         'F'     // forward drive motion
#define GEE         'G'     // hard right turn
#define HAW         'H'     // hard left turn
#define JIB         'J'     // tool angle
#define LEFT        'L'     // left drive motion
#define MULTI       'M'     // multi side drive motion
#define NAK         '_'     // Negative acknowledgement
#define ORIENT      'O'     // Orientation of rover to world
#define PAN         'P'     // camera pan
#define RIGHT       'R'     // right drive motion
#define STOP        'S'     // stop drive motors
#define TILT        'T'     // camera tilt
#define SWEEP       'W'     // sweep sensor pack
#define EXIT        'X'     // exit rover program
#define ZERO        'Z'     // zero steering angle

#define JOYDELAY    5000
/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 ******************************************************************/
#define pressures   true
//#define pressures   false
//#define rumble      true
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int   error = 0;
int   speed;
byte  type = 0;
byte  vibrate = 0;
char  inpt;
int   sphi, splo;
int   xjoy, yjoy;
int   xzero, yzero;
long  joytime;
int   nMotMixL, nMotMixR;
int   nPivSpeed;
float fMotPremixL, fMotPremixR;
float fPivScale;
float fPivYLimit = 32.0;

int   flgup = TRUE;               // key debounce
int   flgdown = TRUE;
int   flgleft = TRUE;
int   flgright = TRUE;
int   exeflag = FALSE;

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
  
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it

  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(error == 0){
    Serial.println("Found Controller, configured successful ");
    }  
  type = ps2x.readType(); 
  if (type == 1) {
      Serial.print("Unknown Controller type found ");
      Serial.print("DualShock Controller found ");
      }
    lcd.setCursor(0, 0);
    lcd.print("Spd: ");
    lcd.setCursor(0, 1);
    lcd.print("Str: ");
    lcd.setCursor(0, 2);
    lcd.print("Hdg: ");
  if (error != 0 && type !=1) {
    lcd.setCursor(0, 0);
    lcd.print("PS/2 controller MIA ");
    while TRUE;                             // infinite loop
    }
    
   joytime = millis() + JOYDELAY;
   xzero = 128;
   yzero = 128;
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

  //----------------------------------------------PS/2 key inputs
  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
  if (ps2x.Button(PSB_SELECT))    // bail out of rover program
    xmit(EXIT);

    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      if (flgup) {
        xmit(FWD);
        flgup = FALSE;
        }
      }
    if (ps2x.ButtonReleased(PSB_PAD_UP))
      flgup = TRUE;
      
    if(ps2x.Button(PSB_PAD_RIGHT)){
      if (flgright) {
        if (ps2x.Button(PSB_L2))
          xmit(GEE);
        else
          xmit(RIGHT);
        flgright = FALSE;
        }
      }
     if (ps2x.ButtonReleased(PSB_PAD_RIGHT))
       flgright = TRUE;
      
     if(ps2x.Button(PSB_PAD_LEFT)){
        if (flgleft) {
       if (ps2x.Button(PSB_L2))
         xmit(HAW);
      else
         xmit(LEFT);
       flgleft = FALSE;
        }
      }
    if (ps2x.ButtonReleased(PSB_PAD_LEFT))
      flgleft = TRUE;
      
    if(ps2x.Button(PSB_PAD_DOWN)){
      if (flgdown) {
        xmit(BACK);
        flgdown = FALSE;
        }
      }
    if (ps2x.ButtonReleased(PSB_PAD_DOWN))
      flgdown = TRUE;

    if (ps2x.Button(PSB_TRIANGLE))
      xmit(COMPASS);
      
    if(ps2x.Button(PSB_L3))              // left joystick button
      xmit(STOP);

    if(ps2x.Button(PSB_L1))              // left upper trigger button
      xmit(ZERO);

    //-----------------------------------------------keypad inputs
    char customkey = keypad.getKey();
    if (customkey) {
      if (customkey == '*')
        exeflag = TRUE;
      else {
        if (exeflag && isDigit(customkey)) {
          xmitnum('E', customkey);
          exeflag = FALSE;
          }
        }
      }

//    if ((millis() - epoch) > 1000) {}
//      xmit('*');                          // were still alive
    epoch = millis();
      
  delay(50);  
  }
