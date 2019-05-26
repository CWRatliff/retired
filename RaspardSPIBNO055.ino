#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

//#define BLUE
#define RED

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
#define DECLINATION 12.1387                               // for Camarillo, CA

Adafruit_BNO055 bno = Adafruit_BNO055(55);
// cal data from BNO example program
#ifdef BLUE
adafruit_bno055_offsets_t caldata = {-13,   -3, -27,      // accel
                                    -375, -195, -75,      // mag
                                    -1,      0,   0,      // gyro
                                    1000,                 // accel radius
                                    429};                 // mag radius
#endif

#ifdef RED
adafruit_bno055_offsets_t caldata = {-22,  -60, -18,      // accel
                                    -473, -439,-235,      // mag
                                    -2,     -1,   2,      // gyro
                                    1000,                 // accel radius
                                    687};                 // mag radius
#endif


void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);           // set to slave mode
  SPI.attachInterrupt();

  bno.begin();
  bno.setSensorOffsets(caldata);
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

  /* Get the floating point data */
  float yaw = event.orientation.x;
  float roll = event.orientation.y;
  float pitch = event.orientation.z;

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

  Serial.print(millis());
  Serial.print(" - Orientation: ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.println(pitch);

  // compose 'O'rientation msg for Pi
  if ((millis() - epoch) > 1000) {
    int hdg = yaw - DECLINATION;            // make into True North;
    if (hdg < 0)
      hdg += 360;
    if (hdg > 360)
      hdg -= 360;
    sprintf(str, "{O%d}.", hdg);
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
