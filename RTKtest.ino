// Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
// 
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada
     
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

uint32_t timer = millis();
long     count = 0;
double  ilatlon;
double  latsum = 0;
double  lonsum = 0;
double  dlatsum = 0;
double  dlonsum = 0;
double  latmin;
double  lonmin;
double latsec;
double lonsec;
double  latvariance = 0;
double  lonvariance = 0;
double  latmean = .337908;          // mean from 190529 patio corner in minutes
double  lonmean = .104772;

void setup() {
  while (!Serial);  // uncomment to have the sketch wait until Serial is ready
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  Wire.begin();
  if (myGPS.begin() == false) {    //Connect to the Ublox module using Wire port
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
    }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  delay(1000);
}  
void loop() // run over and over again
{

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
//    Serial.print("\nTime: ");
//    Serial.print(GPS.hour, DEC); Serial.print(':');
//    Serial.print(GPS.minute, DEC); Serial.print(':');
//    Serial.print(GPS.seconds, DEC); Serial.print('.');
//    Serial.println(GPS.milliseconds);
//    Serial.print("Date: ");
//    Serial.print(GPS.day, DEC); Serial.print('/');
//    Serial.print(GPS.month, DEC); Serial.print("/20");
//    Serial.println(GPS.year, DEC);
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
//    if (GPS.fix) {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      double flat = myGPS.getLatitude();
      flat = flat / 10000000.0;
      latmin = modf(flat, &ilatlon);
      latmin *= 60;
      latsec = modf(latmin, &ilatlon);
      latsec *= 60;
      double flon = myGPS.getLongitude();
      flon = flon / 10000000.0;
      lonmin = modf(flon, &ilatlon);
      lonmin *= 60;
      lonsec = modf(lonmin, &ilatlon);
      lonsec *= -60;
      /*
      Serial.print(" lat/lon ");
      Serial.print(flat,8);
      Serial.print("/");
      Serial.print(flon,8);
      Serial.print("lat/lon min ");
      Serial.print(latmin);
      Serial.print("/");
      Serial.println(lonmin);
      Serial.print("lat/lon sec ");
      Serial.print(latsec, 4);
      Serial.print("/");
      Serial.println(lonsec, 4);
      */
      latsum += latsec;
      lonsum += lonsec;
      dlatsum += latsec;
      dlonsum += lonsec;
      latvariance += pow((latsec - latmean), 2.0);
      lonvariance += pow((lonsec - lonmean), 2.0);
      count++;
      if (count%100 == 0) {
        Serial.print("\nTime: ");
        Serial.print(myGPS.getHour(), DEC); Serial.print(':');
        Serial.print(myGPS.getMinute(), DEC); Serial.print(':');
        Serial.print(myGPS.getSecond(), DEC); Serial.print('.');
//        Serial.println(myGPS.milliseconds);
        Serial.print(">>>>>>> avg lat/lon ");
        Serial.print((latsum/count), 6);
        Serial.print("\"/");
        Serial.print((lonsum/count), 6);
        Serial.print("\" cnt = ");
        Serial.println(count);
        Serial.print(dlatsum/100, 4);
        Serial.print("\"/");
        Serial.print(dlonsum/100, 4);
        Serial.print("\"");
        Serial.print(" Location: ");
        Serial.print(myGPS.latitude); 
//        Serial.print(myGPS.lat);
        Serial.print("/");
        Serial.print(myGPS.longitude);
//        Serial.println(GPS.lon);
/*
        Serial.print("Variances: ");
        Serial.print(latvariance/count, 6);
        Serial.print(" / ");
        Serial.print(lonvariance/count, 6);
        */
        dlatsum = 0;
        dlonsum = 0;
//        }
        
      }
//      Serial.print(", ");
//      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
//      Serial.print("Angle: "); Serial.println(GPS.angle);
//      Serial.print("Altitude: "); Serial.println(GPS.altitude);
//      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
