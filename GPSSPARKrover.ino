#define LCD
#define DELAY 1000
#define MASK 0xff
#define FALSE       0
#define TRUE        (!FALSE)

#include <math.h>
#include <Wire.h> //Needed for I2C to GPS

#ifdef LCD
#include "Adafruit_LiquidCrystal.h"
Adafruit_LiquidCrystal lcd(0);
#endif

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;
char  inpt;
char ibuffer[256];              // traffic from controller to Pi
volatile int itail = 0;
volatile int ihead = 0;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

double latsec;
double  lonsec;
double latcor = 0;
double loncor = 0;

double  ilatlon;
long   gspeed;
long   ghdg;
int   xbflag = FALSE;
//===============================================================
// print text numbers to lcd
char* lcdout(char *s) {
  char  c;
  while (isDigit(*s) || *s == '-' || *s == '.' || *s == ' ') {
    c = *s++;
    lcd.print(c);
    }
  return (s);
  }
//===================================================================
void setup() {

  Serial.begin(115200);
  Wire.begin();
  Serial1.begin(9600);
#ifdef LCD
  lcd.begin(20, 4);
  lcd.clear();
#endif


  if (myGPS.begin() == false) {    //Connect to the Ublox module using Wire port
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

}

//====================================================================
void loop() {
  double trash;
  char    buff[30];
  char*    str;
  
      // read Xbee input
    while (Serial1.available()) {
      inpt = Serial1.read();
      ihead &= MASK;
      if (inpt == '{') {
        ihead = 0;
        continue;
        }
      if (inpt == '}') {
        ibuffer[ihead] = '\0';
        xbflag = TRUE;
        break;
        }
      ibuffer[ihead++] = inpt;
      }
    if (xbflag) {
//      Serial.print("Bcast:");
//      Serial.print(&ibuffer[0]);

      if (ibuffer[0] == 'C') {
        str = &ibuffer[2];
        if (ibuffer[1] == 'T') {
          lcd.setCursor(5, 3);
          lcdout(str);
          latcor = atof(str);
//          Serial.print("latcor:");
//          Serial.print(latcor);
          }
        if (ibuffer[1] == 'N') {
          lcd.setCursor(12, 3);
          lcdout(str);
          loncor = atof(str);
          }
        }
      xbflag = FALSE;
      }
//Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > DELAY) {
    lastTime = millis(); //Update the timer
    long latitude = myGPS.getLatitude();
//    Serial.write("Lat: ");
//    Serial.print(latitude);

    long longitude = myGPS.getLongitude();
//    Serial.write(" Long: ");
//    Serial.print(longitude);
//    Serial.write(" (deg)");
/*
    gspeed = myGPS.getGroundSpeed();     //Returns speed in mm/s
    long lhdg = myGPS.getHeading();
    Serial.print(" hdg:");
    Serial.print(lhdg);
    ghdg = myGPS.getHeading()/100000;       //Returns heading in degrees * 10^-5
*/
    //  lat and long are multiplied by 1e7 to avoid double precision
    //  fixed point lat = (deg)*e7 + (min)*e7/60 + (sec)*e7/3600
    //  reorder to delay double till last operation
    //  assumes lat = 34 deg, 14 min
    long llatsec = (latitude - 342333333) * 36;
    double latsec = (double)llatsec / 100000.0;
    // assumes long = 119 deg, 4 min
    long llongsec = (longitude + 1190666666) * 36;  // long will be minus
    double lonsec = (double)llongsec / 100000.0;
    float latdel = latsec + latcor;
    float londel = lonsec + loncor;       // long increases westward

    Serial.print(" raw:,");
    Serial.print(latsec, 4);
    Serial.print(", ");
    Serial.print(lonsec, 4);
    Serial.print(", Corr:,");
    Serial.print(latdel, 4);
    Serial.print(", ");
    Serial.println(londel, 4);
   

  #ifdef LCD
    char  latstr[10];
    char  lonstr[10];
    dtostrf(latsec, 7, 4, latstr);
    lcd.setCursor(0, 0);
    lcd.print(latstr);
    dtostrf(lonsec, 7, 4, lonstr);
    lcd.setCursor(0,1);
    lcd.print(lonstr);

    dtostrf(latdel, 7, 4, latstr);
    lcd.setCursor(0, 2);
    lcd.print(latstr);
    dtostrf(londel, 7, 4, lonstr);
    lcd.setCursor(10,2);
    lcd.print(lonstr);

//    Serial.println();


//    lcd.setCursor(10, 0);
//    lcd.print(gspeed, 2);
//    double dhdg = (double)hdg / 100000.0;
//    double fdeg = modf(dhdg, &trash);
//    lcd.setCursor(10, 2);
//    lcd.print(ghdg, 2);
/*
    double chdr = atan2(-latdel, -londel*cos(34.14*M_PI/180));
    chdr = chdr * 180/M_PI;
    chdr = (450 - chdr);
    if (chdr > 360)
      chdr -= 360;;
    lcd.setCursor(13, 1);
    lcd.print(chdr);
    */
#endif
   }
}
