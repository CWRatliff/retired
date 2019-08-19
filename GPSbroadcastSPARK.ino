#define LCD
#define DELAY 1000

#include <math.h>
#include <Wire.h> //Needed for I2C to GPS

#ifdef LCD
#include "Adafruit_LiquidCrystal.h"
Adafruit_LiquidCrystal lcd(0);
#endif

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
// from multiday avg 190610
//double lathome = 20.2477;
//double longhome = 6.4108;
// from WM Survey 190809
double lathome = 20.784;
double longhome = 7.8847;

double latsec;
double  lonsec;
double  ilatlon;
float   gspeed;
float   ghdg;
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
  
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > DELAY) {
    lastTime = millis(); //Update the timer
    long latitude = myGPS.getLatitude();
    Serial.write("Lat: ");
    Serial.print(latitude);

    long longitude = myGPS.getLongitude();
    Serial.write(" Long: ");
    Serial.print(longitude);
    Serial.write(" (deg)");

    gspeed = myGPS.getGroundSpeed();     //Returns speed in mm/s
    long lhdg = myGPS.getHeading();
    Serial.print(" hdg:");
    Serial.print(lhdg);
    ghdg = (float) myGPS.getHeading()/10000000;       //Returns heading in degrees * 10^-7

    double latdeg = (double)latitude / 10000000.0;;
    Serial.print(" lat:");
    Serial.print(latdeg);
    double latmin = modf(latdeg, &trash);
    Serial.print(" min:");
    Serial.print(latmin);
    latsec = modf(latmin * 60, &trash);
    latsec *= 60;
    Serial.print(" sec:");
    Serial.print(latsec, 4);
    double londeg = (double)longitude / 10000000.0;
    double lonmin = modf(londeg, &trash);
    lonsec = modf(lonmin * 60, &trash);
    lonsec *= 60;
    lonsec = fabs(lonsec);
    Serial.print(", ");
    Serial.print(lonsec, 4);

  #ifdef LCD
    float latdel = lathome - latsec;
    float londel = lonsec - longhome;       // long increases westward

    Serial.println();

    char  latstr[10];
    char  lonstr[10];
    dtostrf(latsec, 7, 4, latstr);
    lcd.setCursor(0, 0);
    lcd.print(latstr);
    dtostrf(latdel, 7, 4, latstr);
    sprintf(buff, "{CA%s}", latstr) ;
    Serial.write(buff);
    Serial1.write(buff);
    lcd.setCursor(0, 1);
    lcd.print(buff);

    dtostrf(lonsec, 7, 4, lonstr);
    lcd.setCursor(0,2);
    lcd.print(lonstr);
    dtostrf(londel, 6, 4, lonstr);
    sprintf(buff, "{CO%s}", lonstr) ;
    Serial.write(buff);
    Serial1.write(buff);
    lcd.setCursor(0, 3);
    lcd.print(buff);

    lcd.setCursor(10, 0);
    lcd.print(gspeed, 2);
//    double dhdg = (double)hdg / 100000.0;
//    double fdeg = modf(dhdg, &trash);
    lcd.setCursor(10, 2);
    lcd.print(ghdg, 2);

    double chdr = atan2(-latdel, -londel*cos(34.14*M_PI/180));
    chdr = chdr * 180/M_PI;
    chdr = (450 - chdr);
    if (chdr > 360)
      chdr -= 360;;
    lcd.setCursor(13, 1);
    lcd.print(chdr);
#endif
   }
}
