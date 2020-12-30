#define MAGNETTIC_DECLINATION 1.717  // Leiden Netherlands
//#define QUATERNION_FILTER 0  // no filter
//#define QUATERNION_FILTER 1   // Madgwick acceleration magnometer filter
#define QUATERNION_FILTER 2   // Mahony acceleration filter

#include "MPU9250.h"

MPU9255 mpu;
#define LED 13
#define LED2 12
#define SHOW_ACCEL false
#define SHOW_GYRO true
#define SHOW_GYRO_INTEGRATION true
#define SHOW_QUATERNION false
#define SHOW_MAGNOMETER false
#define SHOW_TEMPERATURE false
#define CALIBRATE true

unsigned long output_time;      // milisconden;
unsigned long output_delta=250; // milisconden;

double cum_gx=0;
double cum_gy=0;
double cum_gz=0;
double dt;
unsigned long laatst,nu;

void setup() {
    pinMode(LED, OUTPUT);
    pinMode(LED2, OUTPUT);
    digitalWrite(LED2, HIGH);    // signal leave MPU flat for callibration accel and gyro
    Serial.begin(115200);
    Wire.begin();
    delay(2000);
    //mpu.verbose(true);  // uncomment if you want to see what is happening
    mpu.setup(0x68);  // change to your own address
    digitalWrite(LED, LOW);     
    delay(1000);
    if(CALIBRATE)
    {
      // calibrate anytime you want to
      mpu.calibrateAccelGyro();
      digitalWrite(LED2, LOW);     
      if( QUATERNION_FILTER == 1 )
      {
        digitalWrite(LED, HIGH); // start moving MPU in figure 8 such that all axes have seen all directions
        mpu.calibrateMag();
        digitalWrite(LED, LOW);
      }
      //mpu.printCalibration();

    }
    digitalWrite(LED2, LOW);     
    mpu.setMagneticDeclination(1.0+43.0/60.0);  // Leiden
    showHeaders();
    output_time = millis()+output_delta;
    nu=laatst=micros();
    mpu.start();
}

void loop() {
    if (mpu.update()) {
        if( millis()>output_time) {
           showData();
           output_time+=output_delta;
        }
    }
}

void showData()
{

   mpu.print(
      SHOW_ACCEL,
      SHOW_GYRO,
      SHOW_MAGNOMETER, 
      SHOW_QUATERNION,
      SHOW_GYRO_INTEGRATION, 
      false
      );

  // display the data
  if(SHOW_TEMPERATURE) 
  {
    Serial.print("\t");
    Serial.print(mpu.getTemperature(),6);
  }
  Serial.println("");
}

void showHeaders()
{
  // display the headers
  mpu.printHead(SHOW_ACCEL,
      SHOW_GYRO,
      SHOW_MAGNOMETER, 
      SHOW_QUATERNION,
      SHOW_GYRO_INTEGRATION, 
      false
      );
  if(SHOW_TEMPERATURE) Serial.print("\tt");
  Serial.println("");
}
