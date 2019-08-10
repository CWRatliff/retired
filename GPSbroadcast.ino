//#define SPARK
#define ADA
#define LCD
#define DELAY 1000

#include <math.h>
#include <Wire.h> //Needed for I2C to GPS

#ifdef LCD
#include "Adafruit_LiquidCrystal.h"
Adafruit_LiquidCrystal lcd(0);
#endif

#ifdef SPARK
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

#endif

#ifdef ADA
#include "Adafruit_GPS.h"

// what's the name of the hardware serial port?
#define GPSSerial Serial2

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
     
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true
uint32_t timer = millis();
#endif

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
// from multiday avg 190610
double lathome = 20.2477;
double longhome = 6.4108;
double latsec;
double  lonsec;
double  ilatlon;
//===================================================================
void setup()
{
  Serial.begin(115200);

  Wire.begin();
  Serial1.begin(9600);
#ifdef LCD
  lcd.begin(20, 4);
  lcd.clear();
#endif

#ifdef SPARK
  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
#endif

#ifdef ADA
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
#endif
}

//====================================================================
void loop()
{
  double trash;
  char    buff[30];
  

#ifdef SPARK
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

    long speed = myGPS.getGroundSpeed();     //Returns speed in mm/s
  
    long hdg =  myGPS.getHeading();       //Returns heading in degrees * 10^-7

    long altitude = myGPS.getAltitude();
    Serial.write(" Alt: ");
    Serial.print(altitude);
    Serial.write(" (mm)");

    byte SIV = myGPS.getSIV();
    Serial.write(" SIV: ");
    Serial.print(SIV);

    long acc = myGPS.getPositionAccuracy();
    Serial.write(" acc: ");
    Serial.print(acc);

    double latdeg = (double)latitude / 10000000.0;;
    Serial.print(" lat:");
   Serial.print(latdeg);
    double latmin = modf(latdeg, &trash);
   Serial.print(" miin:");
   Serial.print(latmin);
    double latsec = modf(latmin * 60, &trash);
    latsec *= 60;
   Serial.print(" sec:");
   Serial.print(latsec);
    double londeg = (double)longitude / 10000000.0;
    double lonmin = modf(londeg, &trash);
    double lonsec = modf(lonmin * 60, &trash);
    lonsec *= 60;
    lonsec = fabs(lonsec);
#endif    
#ifdef ADA
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
    }
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    if (GPS.fix) {

      latsec = modf(GPS.latitude, &ilatlon);
      Serial.print(latsec*60, 4);
      lonsec = modf(GPS.longitude, &ilatlon);
      Serial.print(" ");
      Serial.println(lonsec*60, 4);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      }
     }
  }
  #endif
  #ifdef SPARK
    float latdel = lathome - latsec;
    float londel = lonsec - longhome;       // long increases westward

    Serial.println();

    char  latstr[10];
    char  lonstr[10];
    dtostrf(latsec, 7, 4, latstr);
    lcd.setCursor(0, 0);
    lcd.print(latstr);
    dtostrf(latdel, 7, 4, latstr);
    sprintf(buff, "{Ca%s}", latstr) ;
    Serial.write(buff);
    Serial1.write(buff);
    lcd.setCursor(0, 1);
    lcd.print(buff);

    dtostrf(lonsec, 7, 4, lonstr);
    lcd.setCursor(0,2);
    lcd.print(lonstr);
    dtostrf(londel, 6, 4, lonstr);
    sprintf(buff, "{Co%s}", lonstr) ;
    Serial.write(buff);
    Serial1.write(buff);
    lcd.setCursor(0, 3);
    lcd.print(buff);

    lcd.setCursor(10, 0);
    lcd.print(speed);
//    double dhdg = (double)hdg / 100000.0;
//    double fdeg = modf(dhdg, &trash);
    lcd.setCursor(10, 2);
    lcd.print(hdg/1000);

    double chdr = atan2(-latdel, -londel*cos(34.14*M_PI/180));
    chdr = chdr * 180/M_PI;
    chdr = (450 - chdr);
    if (chdr > 360)
      chdr -= 360;;
    lcd.setCursor(13, 1);
    lcd.print(chdr);
  }
#endif
