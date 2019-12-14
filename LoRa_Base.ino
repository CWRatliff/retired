/*
  Send UBX binary commands to enable RTCM sentences on Ublox ZED-F9P module
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 9th, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example does all steps to configure and enable a ZED-F9P as a base station:
    Begin Survey-In
    Once we've achieved 2m accuracy and 300s have passed, survey is complete
    Enable six RTCM messages
    Begin outputting RTCM bytes

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/
#include <SPI.h>

//Radio Head Library:
#include <RH_RF95.h>

// We need to provide the RFM95 module's chip select and interrupt pins to the
// rf95 instance below.On the SparkFun ProRF those pins are 12 and 6 respectively.
RH_RF95 rf95(12, 6);

int LED = 13; //Status LED on pin 13

int packetCounter = 0; //Counts the number of packets sent
long timeSinceLastPacket = 0; //Tracks the time stamp of last packet received
// The broadcast frequency is set to 921.2, but the SADM21 ProRf operates
// anywhere in the range of 902-928MHz in the Americas.
// Europe operates in the frequencies 863-870, center frequency at
// 868MHz.This works but it is unknown how well the radio configures to this frequency:
//float frequency = 864.1;
float frequency = 921.2;


#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

  uint8_t toSend[200];
  int     nbytes = 0;

void setup()
{
  SerialUSB.begin(115200);
  while (!SerialUSB); //Wait for user to open terminal
  SerialUSB.println("Ublox Base station example");

  //Initialize the Radio.
  if (rf95.init() == false) {
    SerialUSB.println("Radio Init Failed - Freezing");
    while (1);
  }
  else {
    // An LED indicator to let us know radio initialization has completed.
    SerialUSB.println("Receiver up!");
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
  rf95.setFrequency(frequency);

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    SerialUSB.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  while (SerialUSB.available()) SerialUSB.read(); //Clear any latent chars in serial buffer
  SerialUSB.println("Press any key to send commands to begin Survey-In");
  while (SerialUSB.available() == 0) ; //Wait for user to press a key

  boolean response = true;
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1); //Enable message 1005 to output through I2C port, message every second
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10); //Enable message every 10 seconds

  //Use COM_PORT_UART1 for the above six messages to direct RTCM messages out UART1
  //COM_PORT_UART2, COM_PORT_USB, COM_PORT_SPI are also available
  //For example: response &= myGPS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART1, 10);

  if (response == true)
  {
    SerialUSB.println("RTCM messages enabled");
  }
  else
  {
    SerialUSB.println("RTCM failed to enable. Are you sure you have an ZED-F9P?");
    while (1); //Freeze
  }

  //Check if Survey is in Progress before initiating one
  response = myGPS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)
  if (response == false)
  {
    SerialUSB.println("Failed to get Survey In status");
    while (1); //Freeze
  }

  if (myGPS.svin.active == true)
  {
    SerialUSB.print("Survey already in progress.");
  }
  else
  {
    //Start survey
    //The ZED-F9P is slightly different than the NEO-M8P. See the Integration manual 3.5.8 for more info.
    //response = myGPS.enableSurveyMode(300, 2.000); //Enable Survey in on NEO-M8P, 300 seconds, 2.0m
    response = myGPS.enableSurveyMode(60, 5.000); //Enable Survey in, 60 seconds, 5.0m
    if (response == false)
    {
      SerialUSB.println("Survey start failed");
      while (1);
    }
    SerialUSB.println("Survey started. This will run until 60s has passed and less than 5m accuracy is achieved.");
  }

  while (SerialUSB.available()) SerialUSB.read(); //Clear buffer

  //Begin waiting for survey to complete
  while (myGPS.svin.valid == false)
  {
    if (SerialUSB.available())
    {
      byte incoming = SerialUSB.read();
      if (incoming == 'x')
      {
        //Stop survey mode
        response = myGPS.disableSurveyMode(); //Disable survey
        SerialUSB.println("Survey stopped");
        break;
      }
    }

    response = myGPS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)
    if (response == true)
    {
      SerialUSB.print("Press x to end survey - ");
      SerialUSB.print("Time elapsed: ");
      SerialUSB.print((String)myGPS.svin.observationTime);

      SerialUSB.print(" Accuracy: ");
      SerialUSB.print((String)myGPS.svin.meanAccuracy);
      SerialUSB.println();
    }
    else
    {
      SerialUSB.println("SVIN request failed");
    }

    delay(1000);
  }
  SerialUSB.println("Survey valid!");

  SerialUSB.println("Base survey complete! RTCM now broadcasting.");

  myGPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3); //Set the I2C port to output UBX and RTCM sentences (not really an option, turns on NMEA as well)
}

void loop()

{
  myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

  delay(250); //Don't pound too hard on the I2C bus
}

//This function gets called from the SparkFun Ublox Arduino Library.
//As each RTCM byte comes in you can specify what to do with it
//Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void SFE_UBLOX_GPS::processRTCM(uint8_t incoming)
{
  toSend[nbytes++] = incoming;
  if (myGPS.rtcmFrameCounter % 16 == 0) {
    SerialUSB.println();
    rf95.send(toSend, nbytes);
    nbytes = 0;
  }
  SerialUSB.print(" ");
  if (incoming < 0x10) SerialUSB.print("0");
  SerialUSB.print(incoming, HEX);
//  uint8_t toSend[] = "AB ";
//  uint8_t toSend[2];
//  toSend[nbytes++] = incoming;
//  rf95.send(toSend, 1);
//  rf95.waitPacketSent();
}
