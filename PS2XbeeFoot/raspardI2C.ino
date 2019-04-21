// relay data between Xbee and RPi
// 190116 
#include <Wire.h>

const int  addrI2C = 0x8;

char  ibuffer[256];
char  obuffer[256];

int  ihead = 0;
int  itail = 0;
int  ohead = 0;
int  otail = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);      // Xbee wired to RX1(19),Tx1(18)
  Wire.begin(addrI2C);    // i2c channel
  Wire.onReceive(receiveEvent);  // register event
  Wire.onRequest(requestCommand);
  }

void loop() {
  char  xchr;
  if(Serial1.available()) {
    xchr = Serial1.read();
    ibuffer[ihead++] = xchr;    // software buffering probably not needed
    if (ihead > 255)
      ihead = 0;
    if (xchr == '}') {
      ibuffer[ihead++] = '\0';
      if (ihead > 255)
        ihead = 0;
      Serial.println(ibuffer);
      }
    }
//  delay(100);
  }

// I2C receive master data
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char c = Wire.read();
    Serial1.write(c);
    }
  }
// I2C send data to master
void requestCommand() {
  unsigned int c;
  if (ihead != itail) {
    Wire.write((char)ibuffer[itail++]);
    if (itail > 255)
      itail = 0;
     }
   }

