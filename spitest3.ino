#include <SPI.h>
char ibuff[] = {"{A}.{B}.{O359}.{C}."};
char obuff[50];
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
  if (c == 0) {             // no char, has to be querry
    if (itail == ihead)
      return;               // nothing to xfer, SPDR still zero
    SPDR = ibuff[itail++];  // get FIFO xbee rcvd data
    itail &= MASK;
    return;
    }
  obuff[ohead++] = c;       // save downloaded char
  ohead &= MASK;
  }
void loop() {
  char* p;
    p = strchr(obuff, '}');
    if (*p == '}') {
      Serial.println(obuff);
      otail = 0;
      ohead = 0;
      }

  }
