/*


  Note: Long/lat are large numbers because they are * 10^7. To convert lat/long
  to something google maps understands simply divide the numbers by 10,000,000. We 
  do this so that we don't have to use floating point numbers.

  Leave NMEA parsing behind. Now you can simply ask the module for the datums you want!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106
*/
#include <math.h>
#include <Wire.h> //Needed for I2C to GPS
#include "Adafruit_LiquidCrystal.h"

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;
#define DELAY 2000
Adafruit_LiquidCrystal lcd(0);

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
// from multiday avg 190610
double lathome = 20.221;
double longhome = 6.326;

//===================================================================
void setup()
{
  Serial.begin(115200);
//  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun Ublox Example");

  Wire.begin();
  Serial1.begin(9600);
  lcd.begin(20, 4);
  lcd.clear();

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

//====================================================================
void loop()
{
  double gspeed;
  long hdg;
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

    long gspeed = myGPS.getGroundSpeed();     //Returns speed in mm/s
    Serial.write(" Spd: ");
    Serial.print(gspeed);  
    
    long hdg =  myGPS.getHeading();       //Returns heading in degrees * 10^-7
    Serial.write(" hdg: ");
    Serial.print(hdg);
    
    long altitude = myGPS.getAltitude();
    Serial.write(" Alt: ");
    Serial.print(altitude);
    Serial.write(" (mm)");

    byte SIV = myGPS.getSIV();
    Serial.write(" SIV: ");
    Serial.print(SIV);

    long acc = myGPS.getHorizontalAccuracy();
    Serial.write(" acc: ");
    Serial.print(acc);
    lcd.setCursor(12, 3);
    lcd.print(acc);

    long llatsec = (latitude - 342333333) * 36;
    double latsec = (double)llatsec / 100000.0;
    long llongsec = (longitude + 1190666666) * 36;  // long will be minus
    double lonsec = (double)llongsec / 100000.0;

    double latdel = lathome - latsec;
    double londel = lonsec + longhome;       // long increases westward
    Serial.write("dels");
    Serial.print(llongsec);
    Serial.print(lonsec);
    Serial.print(londel);
    Serial.println();

    char  latstr[10];
    char  lonstr[10];
    dtostrf(latdel, 7, 4, latstr);
    sprintf(buff, "{CT%s}", latstr) ;
    Serial.write(buff);
    Serial1.write(buff);
    lcd.setCursor(0, 1);
    lcd.print(buff);

    dtostrf(londel, 6, 4, lonstr);
    sprintf(buff, "{CN%s}", lonstr) ;
    Serial.write(buff);
    Serial1.write(buff);
    lcd.setCursor(0, 3);
    lcd.print(buff);

//    dtostrf(gspeed, 7, 3, buff)
//    lcd.setCursor(10, 0);
//    lcd.print(buff);
//    double dhdg = (double)hdg / 100000.0;
//    double fdeg = modf(dhdg, &trash);
    lcd.setCursor(10, 2);
    lcd.print(hdg/100000.0);

    double chdr = atan2(-latdel, -londel*cos(34.14*M_PI/180));
    chdr = chdr * 180/M_PI;
    chdr = (450 - chdr);
    if (chdr > 360)
      chdr -= 360;;
    lcd.setCursor(13, 1);
    lcd.print(chdr);
  }
}
