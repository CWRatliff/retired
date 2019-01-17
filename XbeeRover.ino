// 171019 - TX pin change
// 171021 - joystick
// joystick algorithm adapted from Calvin Hass http://www.impulseadventure.com/elec/

#include <PS2X_lib.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

#define BUFF        256

#define PS2_DAT        11
#define PS2_CMD        10
#define PS2_SEL         9
#define PS2_CLK         8

LiquidCrystal lcd(45, 43, 41, 53, 51, 49, 47);

#define ANGLE       'A'     // tool boom angle
#define BACK        'B'     // reverse drive motion
#define DIRECTION   'D'     // tool base
#define FWD         'F'     // forward drive motion
#define JIB         'J'     // tool angle
#define LEFT        'L'     // left drive motion
#define MULTI       'M'     // multi side drive motion
#define PAN         'P'     // camera pan
#define RIGHT       'R'     // right drive motion
#define STOP        'S'     // stop drive motors
#define TILT        'T'     // camera tilt
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

char  ibuffer[BUFF];
int   itail = 0;
int   ihead = 0;

void btoh(int binp, int &hob, int &lob) {
  hob = binp / 16;
  lob = binp - hob * 16;
 }

// cvt hex digits to uint
int shtoi(char* &st) {
  char  c;
  int   hval = 0;

  for (c = *st; isHexadecimalDigit(c); c = *++st) {
    c = c - '0';
    if (c > 9)
      c -= 7;
    hval = hval * 16 + c;
    }
  return (hval);
  }
  
void xmit(char code, int amt) {
  Serial1.print("{");
  Serial1.print(code);
  Serial1.print(lowByte(amt), HEX);
  Serial1.print("}");
  } 
void setup(){
 
  Serial.begin(115200);

  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Hello World!");
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
}

void loop() {
  /* You must Read Gamepad to get new values and set vibration values
     ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
     if you don't enable the rumble, use ps2x.read_gamepad(); with no values
     You should call this at least once a second
   */  

   if (Serial1.available()) {
    inpt = Serial1.read();
    ibuffer[ihead++] = inpt;
    ihead &= BUFF;
    if (inpt == '}') {
      ibuffer[ihead] = '\0';
      lcd.setCursor(0, 0);
      lcd.print("Speed: ");
      lcd.print(" Angle: ");
      lcd.print(ibuffer[itail]);
      ihead = itail;
      }
    Serial.print(inpt);
    }

  if(error == 1) //skip loop if no controller found
    return; 
  
  if(type == 2){ //Guitar Hero Controller
    return;
    } 
 
  else { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT)) {
      Serial.println("Select is being held");
      }

    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      Serial.print("Up held this hard: ");
      speed = ps2x.Analog(PSAB_PAD_UP);
      Serial.println(speed, DEC);
      btoh(speed, sphi, splo);
      Serial1.print("{F");
      Serial1.print(sphi, HEX);
      Serial1.print(splo, HEX);
      Serial1.print("}");
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
      Serial.print("Right held this hard: ");
      speed = ps2x.Analog(PSAB_PAD_RIGHT);
      Serial.println(speed, DEC);
      Serial1.print("{R");
      Serial1.print(speed, HEX);
      Serial1.print("}");
   }
    if(ps2x.Button(PSB_PAD_LEFT)){
      Serial.print("LEFT held this hard: ");
      speed = ps2x.Analog(PSAB_PAD_LEFT);
      Serial.println(speed, DEC);
      Serial1.print("{L");
      Serial1.print(speed, HEX);
      Serial1.print("}");
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
      Serial.print("DOWN held this hard: ");
      speed = ps2x.Analog(PSAB_PAD_DOWN);
      Serial.println(speed, DEC);
      Serial1.print("{B");
      Serial1.print(speed, HEX);
      Serial1.print("}");
   }   
/*    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if(ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if(ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if(ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if(ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if(ps2x.Button(PSB_TRIANGLE))
        Serial.println("Triangle pressed");        
    }

    if(ps2x.ButtonPressed(PSB_CIRCLE))               //TRUE if button was JUST pressed
      Serial.println("Circle just pressed");
    if(ps2x.NewButtonState(PSB_CROSS))               //TRUE if button was JUST pressed OR released
      Serial.println("X just changed");
    if(ps2x.ButtonReleased(PSB_SQUARE))              //TRUE if button was JUST released
      Serial.println("Square just released");     
*/
/*
    if (ps2x.Button(PSB_L2)) {
       if (ps2x.ButtonPressed(PSB_CROSS)) {
        xmit(ANGLE, -5);
        }
      if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
        xmit(ANGLE, 5);
        }
      if (ps2x.ButtonPressed(PSB_SQUARE)) {
        xmit(DIRECTION, -5);
        }
      if (ps2x.ButtonPressed(PSB_CIRCLE)) {
        xmit(DIRECTION, 5);
        }
     
      }
    else {                    // L1 not pressed
      if (ps2x.ButtonPressed(PSB_CROSS)) {
        xmit(TILT, -5);
        }
      if (ps2x.ButtonPressed(PSB_TRIANGLE)) {
        xmit(TILT, 5);
        }
      if (ps2x.ButtonPressed(PSB_SQUARE)) {
        xmit(PAN, -5);
        }
      if (ps2x.ButtonPressed(PSB_CIRCLE)) {
        xmit(PAN, 5);
        }
      }
    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. 
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC); 
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC); 
      Serial.print(",");
//    Serial.println(ps2x.Analog(PSS_RX), DEC);
      Serial.print(ps2x.Analog(PSS_RX), DEC);
      Serial.print("| adj ");

      yjoy = ps2x.Analog(PSS_RY);   // 0 - top, 255 - bottom
      xjoy = ps2x.Analog(PSS_RX);   // 0 - left, 255 - right
        
      yjoy = yzero - yjoy;          // use latest measured zero for joystick
      xjoy = xjoy - xzero;          // -127 bottom, 127 - top, -127 left, 127 right
      
      if (yjoy < -127) yjoy = -127; // dont exceed limits
      if (yjoy > 127) yjoy = 127;
      if (xjoy < -127) xjoy = -127;
      if (xjoy > 127) xjoy = 127;
      
      Serial.print(yjoy, DEC); 
      Serial.print(", ");
//      Serial.println(xjoy, DEC); 
      Serial.print(xjoy, DEC); 
      
      if (yjoy >= 0) {         // if forward
        fMotPremixL = (xjoy >= 0) ? 127 : (127.0 + xjoy);  // if Rturn, 100% Lmot, else proportional
        fMotPremixR = (xjoy >= 0) ? (127.0 - xjoy) : 127;  // if Lturn, 100% Rmot, else proportional
       }
      else {                    // reverse
        fMotPremixL = (xjoy >= 0) ? (127.0 - xjoy) : 127;  // if Lturn, 100% Rmot, else proportional
        fMotPremixR = (xjoy >= 0) ? 127 : (127.0 + xjoy);  // if Rturn, 100% Lmot, else proportional
       }

      fMotPremixL = fMotPremixL * yjoy/128.0;             // scale using yjoy as throttle
      fMotPremixR = fMotPremixR * yjoy/128.0;

      fPivScale = (abs(yjoy) > fPivYLimit) ? 0.0 : (1.0 - abs(yjoy)/fPivYLimit);

      nMotMixL = (1.0 - fPivScale)*fMotPremixL + fPivScale*xjoy;
      nMotMixR = (1.0 - fPivScale)*fMotPremixR - fPivScale*xjoy;
      
      Serial.print(" xj, yj ");
      Serial.print(xjoy);
      Serial.print(",");
      Serial.print(yjoy);
      Serial.print(" premix L & R ");  
      Serial.print(fMotPremixL);     
      Serial.print(",");
      Serial.print(fMotPremixR);     
      Serial.print(" mix L & R ");  
      Serial.print(nMotMixL, HEX);     
      Serial.print(",");
      Serial.println(nMotMixR, HEX);     
 //      Serial.print(nMotMixR, HEX);   
 //      char c;
 //      c = lowByte(nMotMixR);
 //      Serial.print(", char - ");
 //     Serial.println(c, HEX);  
      Xbee.print("{M");
      Xbee.print(lowByte(nMotMixL), HEX);
      Xbee.print(",");
      Xbee.print(lowByte(nMotMixR), HEX);
      Xbee.print("}");
      joytime = millis() + JOYDELAY;   // reschedule
     }
    else {
      if (millis() > joytime) {
        xzero = ps2x.Analog(PSS_RX);
        yzero = ps2x.Analog(PSS_RY);
        joytime = millis() + JOYDELAY;   // reschedule
        }
      }
    }
    */
  delay(50);  
  }
}
