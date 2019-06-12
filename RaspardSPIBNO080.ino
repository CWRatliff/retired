#include <SPI.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <math.h>

BNO080 myIMU;

unsigned long epoch;
char ibuffer[256];
char obuffer[256];
char str[10];
volatile int itail = 0;
volatile int ihead = 0;
volatile int otail = 0;
volatile int ohead = 0;
volatile int prox;
int oldhdg = 0;

#define MASK 0xff

/* Set the delay between fresh samples */
#define VARIATION 12.1387                               // for Camarillo, CA
#define ROVER     0                             // orientation of sensor to robot


void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);           // set to slave mode
  SPI.attachInterrupt();

  Wire.begin();
  myIMU.begin();
  myIMU.enableRotationVector(450); //Send data update every 450ms
  }
//============================================================
ISR (SPI_STC_vect) {
  byte c = SPDR;
  if (c == 0) {               // no char, has to be query
    if (itail != ihead) {     // something to xfer
      SPDR = ibuffer[itail++];  // get FIFO xbee rcvd data
      itail &= MASK;
      }
    }
  else {
    obuffer[ohead++] = c;       // save downloaded char
    ohead &= MASK;
    }
  }
//============================================================
void loop() {
  char* p;
  char  xchr;
    float qx;
    float qy;
    float qz;
    float qw;
    float quatRadianAccuracy;
    double sinr_cosp;
    double cosr_cosp;
    double roll;
    double sinp;
    double pitch;
    double siny_cosp;
    double cosy_cosp;  
    double yaw;
    int hdg;
    
  if (myIMU.dataAvailable() == true) {
    qx = myIMU.getQuatI();
    qy = myIMU.getQuatJ();
    qz = myIMU.getQuatK();
    qw = myIMU.getQuatReal();
//    quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
/*
    Serial.print(qx, 2);
    Serial.print(F(","));
    Serial.print(qy, 2);
    Serial.print(F(","));
    Serial.print(qz, 2);
    Serial.print(F(","));
    Serial.print(qw, 2);
    Serial.print(F(","));
    Serial.print(quatRadianAccuracy, 2);
*/  
    // convert quaternian to Euler angles
    // roll (x-axis rotation)
    sinr_cosp = +2.0 * (qw * qx + qy * qz);
    cosr_cosp = +1.0 - 2.0 * (qx * qx + qy * qy);
    roll = atan2(sinr_cosp, cosr_cosp);
  
    // pitch (y-axis rotation)
    sinp = +2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
      pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
      pitch = asin(sinp);
  
    // yaw (z-axis rotation)
    siny_cosp = +2.0 * (qw * qz + qx * qy);
    cosy_cosp  = +1.0 - 2.0 * (qy * qy + qz * qz);  
    yaw = atan2(siny_cosp, cosy_cosp) * 180/M_PI;
   
//      Serial.println();
//      delay(400);
    // compose 'O'rientation msg for Pi
    if ((millis() - epoch) > 1000) {
      // Can Dead Men Vote Twice At Elections
      hdg = 360 - yaw;
      hdg = hdg + VARIATION;            // make Mag heading into True
      hdg += ROVER;                         // orientation on rover chassis
      if (hdg < 0)
        hdg += 360;
      if (hdg > 360)
        hdg -= 360;
      if (hdg != oldhdg) {
        sprintf(str, "{O%d}.", hdg);
        for (char *p = &str[0]; *p; p++) {
          ibuffer[ihead++] = *p;
          ihead &= MASK;
          Serial.print(p);
          }
        oldhdg = hdg;
        }
      /*
      Serial.print("yaw:");
      Serial.print(hdg);
      Serial.print(" pitch: ");
      Serial.print(pitch*180.0/M_PI, 2);
      Serial.print(" roll: ");
      Serial.print(roll*180.0/M_PI, 2);
      Serial.println(str);
      */
      epoch = millis();
      }
    
    // read Xbee input and upload to Pi
    while (Serial1.available()) {
      xchr = Serial1.read();
      Serial.println(xchr);
      ibuffer[ihead++] = xchr;
      ihead &= MASK;
      if (xchr == '}') {
        ibuffer[ihead++] = 0;
        ihead &= MASK;
        break;
        }
      }
      
    // if any data waiting in output buffer, transmit via Xbee    
    if (ohead != otail) {
      if (Serial1.availableForWrite()) {
        Serial1.write(obuffer[otail++]);
        otail &= MASK;
        }
      }
    }
  } 
