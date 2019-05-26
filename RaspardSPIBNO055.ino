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

Adafruit_BNO055 bno = Adafruit_BNO055(55);
// cal data from BNO example program
#ifdef BLUE
adafruit_bno055_offsets_t caldata = {-13,   -3, -27,      // accel
                                    -1,      0,   0,      // gyro
                                    -375, -195, -75,      // mag
                                    1000,                 // accel radius
                                    429};                 // mag radius
#endif

#ifdef RED
adafruit_bno055_offsets_t caldata = {-13,   -3, -27,      // accel
                                    -1,      0,   0,      // gyro
                                    -375, -195, -75,      // mag
                                    1000,                 // accel radius
                                    429};                 // mag radius
#endif

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);           // set to slave mode
  SPI.attachInterrupt();

  bno.begin();
  bno.setExtCrystalUse(true);
  bno.setSensorOffsets(caldata);
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
  float pitch = event.orientation.y;
  float roll = event.orientation.z;


  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

  int inthead = yaw;


  Serial.print(millis());
  Serial.print(" - Orientation: ");
  Serial.print(yaw);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);

/*
  imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  Serial.print("Mag vector: ");
  Serial.print(magnet.x());
  Serial.print(", ");
  Serial.print(magnet.y());
  Serial.print(", ");
  Serial.print(magnet.z());

  double ang = atan2(magnet.y(), magnet.x()) *180 / M_PI;
  Serial.print(",  atan : ");
  Serial.print(ang);
  ang = yaw - ang;
  if (ang < 0)
    ang += 360;
  if (ang > 360)
    ang -= 360;
  Serial.print(" hdg : ");
  Serial.println(ang);
  inthead = ang;
  */
  delay(300);
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
