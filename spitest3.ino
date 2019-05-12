#include <SPI.h>
char ibuff[] = {"{A}.{B}.{O359}.{C}."};
char obuff[50];
volatile int itail = 0;
volatile int ihead = sizeof(ibuff);
volatile int otail = 0;
volatile int prox;

void setup() {
  Serial.begin(115200);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);        // set to slave mode
  SPI.attachInterrupt();
  }


ISR (SPI_STC_vect) {
  byte c = SPDR;
  if (itail < ihead)      // send char if buffer not empty
    SPDR = ibuff[itail++];
  else
    SPDR = 0;
  if (c)                  // did we get anything?
    obuff[otail++] = c;
  else
    prox = true;
  }

void loop() {
  if (prox) {
    Serial.println(obuff);
    otail = 0;
    prox = false;
    Serial.print("tail ");
    Serial.print(itail);
    Serial.print(" head  ");
    Serial.println(ihead);
    if (itail >= ihead)
      itail = 0;
    }
    if (itail >= ihead)
      itail = 0;
  }
