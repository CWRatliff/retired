#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

unsigned long epoch;
char ibuffer[256];
char obuffer[256];
char str[10];
volatile int itail = 0;
volatile int ihead = 0;
volatile int otail = 0;
volatile int ohead = 0;
volatile int prox;

#define MASK 0xff
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);           // set to slave mode
  SPI.attachInterrupt();
  bno.setExtCrystalUse(true);

  }
//============================================================
ISR (SPI_STC_vect) {
  byte c = SPDR;
  if (c == 0) {               // no char, has to be querry
    if (itail != ihead) {     // something to xfer
      SPDR = ibuffer[itail++];  // get FIFO xbee rcvd data
      itail &= MASK;
      }
    }
  else {
    obuffer[ohead++] = c;       // save downloaded char
    ohead &= MASK;
    }
  }
//============================================================
void loop() {
  char* p;
  char  xchr;

  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
    float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw();
  int inthead = heading;

/*
  Serial.print(millis());
  Serial.print(" - Orientation: ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);
*/ 
/***************************************8
  if(ohead > 0)
    p = strchr(obuffer, '}');
    if (*p == '}') {
      Serial.println(obuffer);
      *p = ' ';
      otail = 0;
      ohead = 0;
      }
 *********************************************/
  // compose 'O'rientation msg for Pi
  if ((millis() - epoch) > 1000) {
    sprintf(str, "{O%d}.", inthead);
    for (char *p = &str[0]; *p; p++) {
      ibuffer[ihead++] = *p;
      ihead &= MASK;
      }
    Serial.println(str);
    epoch = millis();
    }
    
  // read Xbee input and upload to Pi
  while (Serial1.available()) {
    xchr = Serial1.read();
    ibuffer[ihead++] = xchr;
    ihead &= MASK;
    if (xchr == '}') {
      ibuffer[ihead++] = 0;
      ihead &= MASK;
//      msgcount++;
      }
    }
    
  // if any data waiting in output buffer, transmit via Xbee    
  if (ohead != otail) {
    if (Serial1.availableForWrite()) {
      Serial1.write(obuffer[otail++]);
      otail &= MASK;
      }
    }
  } 
