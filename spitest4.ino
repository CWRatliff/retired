#include <SPI.h>
char ibuff[256] = {"{A}.{B}.{O359}.{C}.{CO123.45678}.{R4}.{F3}"};
char obuff[256];
volatile int itail = 0;
volatile int ihead = sizeof(ibuff);
volatile int otail = 0;
volatile int ohead = 0;
volatile int prox;

#define MASK 0xff

void setup() {
  Serial.begin(115200);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);        // set to slave mode
  SPI.attachInterrupt();
  }
ISR (SPI_STC_vect) {
  byte c = SPDR;
//  Serial.print(" I:");
//  Serial.print(c, HEX);
  if (c == 0) {             // no char, has to be querry
    if (itail != ihead) {   // somethingthing to xfer
      c = ibuff[itail++];  // get FIFO xbee rcvd data
      SPDR = c;
//      Serial.print(" O:");
//      Serial.print(c, HEX);
      itail &= MASK;
      }
    }
  else {
    obuff[ohead++] = c;       // save downloaded char
    ohead &= MASK;
    }
  }
void loop() {
  char* p;
  if(ohead > 0)
    p = strchr(obuff, '}');
    if (*p == '}') {
      Serial.println(obuff);
      *p = ' ';
      otail = 0;
      ohead = 0;
      }

  }
