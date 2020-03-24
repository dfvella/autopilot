#include "imu.h"

#define CALIBRATE_ACCELEROMETER

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
  Serial.print(imu.angle_x());
  Serial.print(" ");
  Serial.println(imu.angle_y());
}
