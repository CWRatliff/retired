#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_Sensor.h>
//#include <Mahony.h>
#include <Madgwick.h>

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
/*
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each board/environment.
// See the image in this sketch folder for the values used
// below.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -7.93F, -16.91F, -131.57F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.966, -0.037, -0.030 },
                                    { -0.037,  1.002, -0.001 },
                                    { -0.030, -0.001,  1.036 } };

float mag_field_strength        = 42.83F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Mahony is lighter weight as a filter and should be used
// on slower systems
//Mahony filter;
Madgwick filter;
*/
// relay data between Xbee and RPi
// 190116 
//const int  addrI2C = 0x8;
// 190515 switch from I2C to SPI
unsigned long epoch;
char  ibuffer[256];
char  obuffer[256];

int  ihead = 0;
int  itail = 0;
int  ohead = 0;
int  otail = 0;
int   msgcount = 0;
//char  str[100];
//int   strflag = 0;

#define MASK  0xff;

//========================================================================
void setup() {
  pinMode(MISO, OUTPUT);
  SPCR |- _BV(SPE);
  SPI.attachInterrupt();
  epoch = millis();
  Serial.begin(115200);
  Serial1.begin(9600);      // Xbee wired to RX1(19),Tx1(18)
//  Wire.begin(addrI2C);    // i2c channel
//  Wire.onReceive(receiveEvent);  // register event
//  Wire.onRequest(requestCommand);

  // Wait for the Serial Monitor to open (comment out to run without Serial Monitor)
  // while(!Serial);

  Serial.println(F("Adafruit AHRS Fusion Example")); Serial.println("");
/*
  // Initialize the sensors.
  if(!gyro.begin()){
    // There was a problem detecting the gyro ... check your connections
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
 Serial.println("");
 delay(1000);
    while(1);
  }
Serial.println("after gyro");
  if(!accelmag.begin(ACCEL_RANGE_4G)){
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
 Serial.println("");
 while(1);
  }

  // Filter expects 70 samples per second
  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  filter.begin(10);
  */
  }
//========================================================================
void loop(void) {
  char  str[20];
  char  xchr;
/*--------------------------------
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  // Get new data samples
  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  // Print the orientation filter output
  // Note: To avoid gimbal lock you should read quaternions not Euler
  // angles, but Euler angles are used here since they are easier to
  // understand looking at the raw values. See the ble fusion sketch for
  // and example of working with quaternion data.
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

int inthead = ihead;    // need something to send>>>>>>>>>>>>>>>>>>>>>>>>>
  // compose 'O'rientation msg for Pi
  if ((millis() - epoch) > 1000) {
    sprintf(str, "{O%d}", inthead);
    for (char *p = &str[0]; *p; p++) {
      ibuffer[ihead++] = *p;
      ihead &= MASK;
      }
    msgcount++;

    Serial.println(str);
    epoch = millis();
    }

    delay(10);

  // read Xbee input and upload to Pi
  while (Serial1.available()) {
    xchr = Serial1.read();
    ibuffer[ihead++] = xchr;
    ihead &= MASK;
    if (xchr == '}') {
      ibuffer[ihead++] = 0;
      ihead &= MASK;
      msgcount++;
      }
    }
    
  // if any data waiting in output buffer, transmit via Xbee    
  while (ohead != otail) {
    if (Serial1.availableForWrite()) {
      Serial1.write(obuffer[itail++]);
      itail &= MASK
      }
    }
  }
//========================================================================
// ISR for SPI slave listener
ISR (SPI_STC_vect) {
  byte c = SPDR;
  if (c == 0) {               // no char, must be querry
    if (itail != ihead) {     // anything to xfer?
      SPDR = ibuffer[itail++];
      itail &= MASK;
      }
    }
  else {                      // rcv data from master
    obuffer[ohead++] = c;
    ohead &= MASK;
    }
  }

/*  
//========================================================================
// I2C receive master data
void receiveEvent(int howMany) {
  while (Wire.available()) {
    char c = Wire.read();
    Serial1.write(c);
    }
  }
//========================================================================
// I2C send data to master
void requestCommand() {
  unsigned int c;
  if (ihead != itail) {
    Wire.write((char)ibuffer[itail++]);
    if (itail > 255)
      itail = 0;
     }
   }
*/
