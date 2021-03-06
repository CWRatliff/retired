//190823 - added '.'s to msgs - seems to be SPI timing need
//191019 - revised lat/long arithmetic to avoid rounding loss of accuracy
#include <SPI.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <math.h>
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS

SFE_UBLOX_GPS myGPS;
BNO080 myIMU;

unsigned long epoch;
char ibuffer[256];              // traffic from controller to Pi
volatile int itail = 0;
volatile int ihead = 0;

char    obuffer[256];              // traffic from Pi to controller
volatile int otail = 0;
volatile int ohead = 0;
char    str[25];

int     oldhdg = 0;

#define MASK 0xff

//==========================================================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);        // XBee
  pinMode(MISO, OUTPUT);      // SPI
  SPCR |= _BV(SPE);           // set to slave mode
  SPI.attachInterrupt();

  Wire.begin();
  myIMU.begin();
  myIMU.enableRotationVector(450); //Send data update every 450ms
  myGPS.begin();
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

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
  double trash;
  int hdg;
  
  // read Xbee input and upload to Pi
  while (Serial1.available()) {
    xchr = Serial1.read();
//      Serial.println(xchr);
    ibuffer[ihead++] = xchr;
    ihead &= MASK;
    if (xchr == '}') {
      ibuffer[ihead++] = 0;
      ihead &= MASK;
      break;
      }
    }
      
  // if any data waiting in output buffer, transmit via Xbee    
  while (ohead != otail) {
    if (Serial1.availableForWrite()) {
      Serial.write(obuffer[otail]);
      Serial1.write(obuffer[otail++]);
      otail &= MASK;
      }
    else {
      Serial.println("buffer");
      break;
      }
    } // endwhile

  if ((millis() - epoch) > 1000) {
    epoch = millis();
    
    if (myIMU.dataAvailable() == true) {
      qx = myIMU.getQuatI();
      qy = myIMU.getQuatJ();
      qz = myIMU.getQuatK();
      qw = myIMU.getQuatReal();
  //    quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
  /*
      Serial.print(qx, 3);
      Serial.print(F(","));
      Serial.print(qy, 3);
      Serial.print(F(","));
      Serial.print(qz, 3);
      Serial.print(F(","));
      Serial.print(qw, 3);
      Serial.print(F(","));
  //    Serial.print(quatRadianAccuracy, 2);
      Serial.println();
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

      // compose 'O'rientation msg for Pi
      hdg = 450 - yaw;                  // cvt math CCW x-axis to azimuth
      hdg = hdg % 360;
      }

    if (hdg != oldhdg) {
      sprintf(str, "{O%d}..", hdg);
      for (char *p = &str[0]; *p; p++) {
        ibuffer[ihead++] = *p;
        ihead &= MASK;
        }
      oldhdg = hdg;
      Serial.println(str);
      }
/*
      Serial.print("yaw:");
      Serial.print(yaw);
      Serial.print(" pitch: ");
      Serial.print(pitch*180.0/M_PI, 2);
      Serial.print(" roll: ");
      Serial.print(roll*180.0/M_PI, 2);
      Serial.println(str);
*/


    long latitude = myGPS.getLatitude();
    long longitude = myGPS.getLongitude();

      
    //  lat and long are multiplied by 1e7 to avoid double precision
    //  fixed point lat = (deg)*e7 + (min)*e7/60 + (sec)*e7/3600
    //  reorder to delay double till last operation
    //  assumes lat = 34 deg, 14 min
    long llatsec = (latitude - 342333333) * 36;
    double latsec = (double)llatsec / 100000.0;
    // assumes long = 119 deg, 4 min
    long llongsec = (longitude + 1190666666) * 36;  // long will be minus
    double lonsec = (double)llongsec / 100000.0;

    lonsec = fabs(lonsec);
    latsec = fabs(latsec);

    char latstr[15];
    dtostrf(latsec, 7, 4, latstr);
    sprintf(str, "{LT%s}....", latstr);
    Serial.println(str);
    for (char *p = &str[0]; *p; p++) {
       ibuffer[ihead++] = *p;
       ihead &= MASK;
       }

    char lonstr[15];
    dtostrf(lonsec, 6, 4, lonstr);
    sprintf(str, "{LN%s}....", lonstr);
    Serial.println(str);
    for (char *p = &str[0]; *p; p++) {
       ibuffer[ihead++] = *p;
       ihead &= MASK;
       }
    }
  } 
