#include <Wire.h>
//#include <SoftwareSerial.h>

//#define  Rx  12      // Mega compatible (with jumper)
//#define  Tx  3

//SoftwareSerial Xbee(Rx, Tx);

const int ledPin = 13; //led on board
const int Low = 0;
const int  addrI2C = 0x8;

char  ibuffer[256];
char  obuffer[256];

int  ihead = 0;
int  itail = 0;
int  ohead = 0;
int  otail = 0;

void setup() {
  Serial.begin(115200);
//  Xbee.begin(9600);
  Serial1.begin(9600);
  Wire.begin(addrI2C);    // i2c channel
  Wire.onReceive(receiveEvent);  // register event
  Wire.onRequest(requestCommand);
  
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, Low); //turn it off
  }

void loop() {
  char  xchr;
//  if(Xbee.available()) {
//    xchr = Xbee.read();
  if(Serial1.available()) {
    xchr = Serial1.read();
    ibuffer[ihead++] = xchr;
    if (ihead > 255)
      ihead = 0;
    if (xchr == '}') {
      ibuffer[ihead++] = '\0';
      if (ihead > 255)
        ihead = 0;
      Serial.println(ibuffer);
//      Wire.beginTransmission(addrI2C);
      }
    }
  delay(100);
  }

// I2C receive master data
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char c = Wire.read();
    Serial1.write(c);
    digitalWrite(ledPin, c);
    }
  }
// I2C send data to master
void requestCommand() {
  unsigned int c;
  if (ihead != itail) {
//    Wire.write('X');
//    Wire.write(87);
    Wire.write((char)ibuffer[itail++]);
//    c = ibuffer[itail++] & 0x7f;
//    c = 'A';
//    Wire.write(c);
//    Wire.write(ibuffer[itail++], 1);
    if (itail > 255)
      itail = 0;
     }
   }

