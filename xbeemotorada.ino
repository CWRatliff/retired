// 171102
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
//#include <Servo.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
#define HALT  0
#define FWD   1
#define REV   2
#define MDELAY  200
Adafruit_DCMotor *wheelRR = AFMS.getMotor(1);
Adafruit_DCMotor *wheelRF = AFMS.getMotor(2);
Adafruit_DCMotor *wheelLF = AFMS.getMotor(3);
Adafruit_DCMotor *wheelLR = AFMS.getMotor(4);

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver();
/*Servo camPan;
Servo camTilt;
Servo armBase;
Servo armBoom;
Servo armTool;*/
#define CAMpan    8
#define CAMtilt   9
#define ARMbase   10
#define ARMboom   11
#define ARMtool   12

LiquidCrystal lcd(8, 9, 10, 4, 5, 6, 7);

#define Rx  12 // to 3 on xbee adaptor (on Mega, must be 10, 11, 12, ...
#define Tx  3 // to 4 on xbee adaptor
SoftwareSerial Xbee(Rx, Tx);


char  ibuffer[10];        // xbee input buffer
int   bndx;
int   oldL = 0;           // motor speeds from last loop
int   oldR = 0;
unsigned long  ps2time;   // time of last ps/2 message
int   base;
int   boom;
int   pan;
int   tilt;
int   tool;
//====================================================================
// cvt hex digits to int (unsigned)
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
//=======================================================================
// timer delay
void wdelay(unsigned long timer) {
  if (timer <= millis())        // timer has already gone off
    return;
  delay (timer - millis());
  }
//=======================================================================
// run wheel motors on Adafruit motor controllers using "speed" as direction and magnitude
void Wheel(Adafruit_DCMotor *wheelF, Adafruit_DCMotor *wheelR, int speed) {

  if (speed == 0) {
    wheelF->run(RELEASE);
    wheelR->run(RELEASE);
    return;
    }
  if (speed > 0) {
    wheelF->setSpeed(speed);
    wheelR->setSpeed(speed);
    wheelF->run(FORWARD);
    wheelR->run(FORWARD);
    }
  else {
    wheelF->setSpeed(-speed);
    wheelR->setSpeed(-speed);
    wheelF->run(BACKWARD);
    wheelR->run(BACKWARD);
    }
  }
//==================================================================================
// compute delay time if necessary due to state change
// intended motor (Lynxmotion gear head motor) must be stopped before changing direction
unsigned long delaySet(int& oldstate, int speed) {
unsigned long  timer;

  timer = 0;
  if (speed == 0) {
      if (oldstate == FWD || oldstate == REV) // if just stopping after motion
        timer = millis() + MDELAY;            // then delay until MDELAY from now
      oldstate = HALT;
      }
   else if (speed > 0) {
      if (oldstate == REV)                    // change to FWD from REV, delay
        timer = millis() + MDELAY;
      oldstate = FWD;
       }
    else {
      if (oldstate == FWD)                    // change to REV from FWD, delay
       timer = millis() + MDELAY;
      oldstate = REV;
    }
  return (timer);
  }
//====================================================================
void setup() {

  Serial.begin(9600);
  Serial.print("Hello world");
  AFMS.begin();  // create with the default frequency 1.6KHz
  servo.begin();
  servo.setPWMFreq(60);
  
  Wheel(wheelLF, wheelLR, 0);
  Wheel(wheelRF, wheelRR, 0);

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
  Xbee.begin(9600);

  for (int f = 100; f < 450; f++)
    servo.setPWM(CAMtilt, 0, f); 
  delay(1000);
  for (int f = 450; f > 100; f--)
    servo.setPWM(CAMtilt, 0, f); 
  servo.setPWM(CAMtilt, 0, 350);
  tilt = 350;
  
  for (int f = 100; f < 450; f++)
    servo.setPWM(CAMpan, 0, f); 
  delay(1000);
  for (int f = 450; f > 100; f--)
    servo.setPWM(CAMpan, 0, f); 
  servo.setPWM(CAMpan, 0, 450);
/*  camPan.attach(22);
  camPan.write(130);
  camTilt.attach(24);
  camTilt.write(90);
  tilt = 90;
  armBase.attach(26);
  armBase.write(0);
  armBoom.attach(28);
  armBoom.write(90);
  armTool.attach(30);
  armTool.write(45);*/
  
  delay(500); 
  bndx = 0;
  ibuffer[0] = '\0';
  }
//====================================================================
void loop() {
  char  cin;
  char  cmd;
  char* str;
  int   motL;             // left and right motor speed
  int   motR;
  int   motorspeed;
  int   param;
  unsigned long  timerL;  // delays to stop motors before reversing
  unsigned long  timerR;
  
//  if (Xbee.available()) {
    if (!Xbee.available())
      return;
      
    cin = Xbee.read();
    ibuffer[bndx++] = cin;
    ibuffer[bndx] = '\0';
    if (cin != '}')
      return;
   
    lcd.setCursor(0, 0);
    lcd.print(ibuffer);
    lcd.print(" ");
//  Xbee.print(ibuffer);        // sending Xbee could error check
  
  if (ibuffer[0] == '{') {
    ps2time = millis();       // save time of most recent ps/2 command
    lcd.setCursor(0, 0);
    lcd.print(ibuffer);
    lcd.print(" ");
    lcd.setCursor(0, 2);
    cmd = ibuffer[1];
    str = &ibuffer[2];
      
    // joystick - command "{Mll,rr}" ll & rr are 1-2 hex chars, 7 bit plus sign bit
    if (cmd == 'M') {
      motL = shtoi(str);
      str++;
      motR = shtoi(str);
      if (motL > 127) motL -= 256;  // sign extend signed byte to signed int
      if (motR > 127) motR -= 256;
      motL *= 2;                    // cvt to -255 to +255
      motR *= 2;
      lcd.print(motL);
      lcd.print(" ");
      lcd.print(motR);
      lcd.print(" ");
      timerL = delaySet(oldL, motL);
      timerR = delaySet(oldR, motR);
      if (timerL < timerR) {            // delay smaller delay first
        if (motL)                       // only delay if not a HLT
          wdelay(timerL);
        Wheel(wheelLF, wheelLR, motL);
        if (motR)
          wdelay(timerR);
        Wheel(wheelRF, wheelRR, motR);
        }
      else {
        if (motR)
          wdelay(timerR);
        Wheel(wheelRF, wheelRR, motR);
        if (motL)
          wdelay(timerL);
        Wheel(wheelLF, wheelLR, motL);
        }
      motorspeed = max(motL, motR);
      }
    else if (cmd == 'F') {
      motorspeed = shtoi(str);
      Wheel(wheelLF, wheelLR, motorspeed);
      Wheel(wheelRF, wheelRR, motorspeed);
      lcd.print("Fwd ");
      }
    else if (cmd == 'L') {
      motorspeed = shtoi(str);
      Wheel(wheelLF, wheelLR, -motorspeed);
      Wheel(wheelRF, wheelRR, motorspeed);
      lcd.print("Lft ");
      }
    else if (cmd == 'R') {
      motorspeed = shtoi(str);
      Wheel(wheelLF, wheelLR, motorspeed);
      Wheel(wheelRF, wheelRR, -motorspeed);
      lcd.print("Rgt ");
      }
    else if (cmd == 'B') {
      motorspeed = shtoi(str);
      Wheel(wheelLF, wheelLR, -motorspeed);
      Wheel(wheelRF, wheelRR, -motorspeed);
      lcd.print("Aft ");
      }
    else if (cmd == 'T') {
      param = shtoi(str);
      if (param > 127) param -= 256;
      tilt = tilt + param;
//      camTilt.write(tilt);
      Serial.print("Tilt ");
      Serial.print(ibuffer);
      Serial.print(" tlt ");
      Serial.print(tilt);
      Serial.print(" par ");
      Serial.println(param);
      servo.setPWM(CAMtilt, 0, tilt);
      delay(50);
      lcd.print("Tilt ");
      lcd.print(tilt, DEC);
      lcd.print("   ");
      }
    else {
      lcd.setCursor(0, 3);
      lcd.print(millis());
      lcd.print(" ");
      lcd.print(ibuffer);
      }
    lcd.print(motorspeed, DEC);
    lcd.print("   ");
    }
  bndx = 0;           // reset input buffer
  ibuffer[0] = '\0';
  lcd.setCursor(0,0);

  if ((millis() - ps2time) > 200) {
    lcd.setCursor(0, 1);
    lcd.print("Halt       ");
    ps2time = millis();
    Wheel(wheelLF, wheelLR, 0);
    Wheel(wheelRF, wheelRR, 0);
    }
  }


