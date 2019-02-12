//190210
// joystick algorithm adapted from Calvin Hass http://www.impulseadventure.com/elec/

#include <PS2X_lib.h>
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"
//#include <LiquidCrystal.h>

#define FALSE       0
#define TRUE        !FALSE
#define BUFF        256

#define PS2_DAT        11
#define PS2_CMD        10
#define PS2_SEL         9
#define PS2_CLK         8

Adafruit_LiquidCrystal lcd(0);

//  Xbee  Mega
//  2     19 rx1
//  3     18 tx1

#define ACK         '*'     // Acknowledge
#define ANGLE       'A'     // tool boom angle
#define BACK        'B'     // reverse drive motion
#define DIRECTION   'D'     // tool base
#define FWD         'F'     // forward drive motion
#define  GEE     'G'   // hard right turn
#define HAW     'H'   // hard left turn
#define JIB         'J'     // tool angle
#define LEFT        'L'     // left drive motion
#define MULTI       'M'     // multi side drive motion
#define NAK         '_'     // Negative acknowledgement
#define PAN         'P'     // camera pan
#define RIGHT       'R'     // right drive motion
#define STOP        'S'     // stop drive motors
#define TILT        'T'     // camera tilt
#define SWEEP       'W'     // sweep sensor pack
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

char  ibuffer[BUFF];              // Xbee software input buffer
int   itail = 0;
int   ihead = 0;

int   epoch;                      // last xmission time
//================================================================
// 1-byte int to two hex bytes
void btoh(int binp, int &hob, int &lob) {
  hob = binp / 16;
  lob = binp - hob * 16;
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
void xmit1(char code, int amt) {
  Serial1.print("{");
  Serial1.print(code);
  Serial1.print(lowByte(amt), HEX);
  Serial1.print("}");
  epoch = millis();
  } 
//====================================================================
void setup(){
 
  Serial.begin(115200);

  lcd.begin(20, 4);
  lcd.clear();
  Serial1.begin(9600);
  
  delay(300);  //added delay to give wireless ps2 module some time to startup, before configuring it

  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(error == 0){
    Serial.println("Found Controller, configured successful ");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  
//  Serial.print(ps2x.Analog(1), HEX);
  
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      lcd.setCursor(0, 0);
      lcd.print("Speed: ");
      lcd.setCursor(0, 1);
      lcd.print("Angle: ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
  case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }
   joytime = millis() + JOYDELAY;
   xzero = 128;
   yzero = 128;
   xmit('*');             // were alive
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
  
   while (Serial1.available()) {               // Xbee input from motor control RPi
    inpt = Serial1.read();
    if (inpt == '{') {
      ihead = 0;
      continue;
      }
    if ((ihead >= 0) && (inpt != '}')) {
      ibuffer[ihead++] = inpt;
      continue;
      }
    ibuffer[ihead] = '\0';
    lcd.setCursor(7, 0);
    str = &ibuffer[1];
    spd = shtoi(str);
    if (spd > 127)
      spd = spd - 256;
    lcd.print(spd, DEC);
    lcd.print("   ");
    str = (ibuffer[2] == ',') ? &ibuffer[3] : &ibuffer[4];
    ang = shtoi(str);
    if (ang > 127)
      ang = ang - 256;
    lcd.setCursor(7, 1);
    lcd.print(ang, DEC);
    lcd.print("   ");
//    Serial.println(ibuffer);
    }

  if(error == 1) //skip loop if no controller found
    return; 
  
  if(type == 2){ //Guitar Hero Controller
    return;
    } 
 
  else { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
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
      
    if(ps2x.Button(PSB_L3))              // left joystick button
      xmit(STOP);

    if(ps2x.Button(PSB_L1))              // left upper trigger button
      xmit(ZERO);

    if ((millis() - epoch) > 1000)
      xmit('*');                          // were still alive
      
  delay(50);  
  }
}
