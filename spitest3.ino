#include <SPI.h>
char ibuff[50] = {"\{A\}.\{B\}.\{O359\}.\{C\}."};
char obuff[50];
volatile int itail = 0;
volatile int ihead = sizeof(ibuff);
volatile int prox;

void setup() {
  Serial.begin(115200);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPI.attachInterrupt();
  }


ISR (SPI_STC_vect) {
  byte c = SPDR;
  if (itail < ihead)
    SPDR = ibuff[itail];
  else
    SPDR = 0;
  if (c)
    obuff[itail++] = c;
  else
    prox = true;
  }

void loop() {
  if (prox) {
    Serial.println(obuff);
    prox = false;

    }
  }
