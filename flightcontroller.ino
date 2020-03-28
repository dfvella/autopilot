#include "imu.h"

//#define CALIBRATE_ACCELEROMETER

Imu imu(13);

void setup() {
  Serial.begin(9600);
  #ifdef CALIBRATE_ACCELEROMETER
  imu.calibrate_accel();
  #endif 
  #ifndef CALIBRATE_ACCELEROMETER
  imu.calibrate();
  #endif
}

void loop() {
  imu.run();
  Serial.print(imu.get_roll());
  Serial.print(" ");
  Serial.print(imu.get_pitch());
  Serial.print(" ");
  Serial.println(imu.get_yaw());
}
