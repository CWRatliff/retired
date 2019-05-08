#include <SPI.h>
char buff[50];
volatile byte indx;
volatile boolean prox;

void setup() {
  Serial.begin(115200);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  indx = 0;
  prox = false;
  buff[0] = 4;
  buff[1] = '{';
  buff[2] = 'O';
  buff[3] = '7';
  buff[4] = '}';
  buff[5] = '\r';
  SPI.attachInterrupt();
  }

ISR (SPI_STC_vect) {
  byte c = SPDR;
  if (indx < sizeof buff) {
    SPDR = buff[indx];
    buff[indx++] = c;
    if (c == '\r')
      prox = true;
    }
  }

void loop() {
  if (prox) {
    Serial.println(buff);
    indx = 0;
    prox = false;
  buff[0] = 4;
  buff[1] = '{';
  buff[2] = 'O';
  buff[3] = '7';
  buff[4] = '}';
  buff[5] = '\r';
    }
  }
