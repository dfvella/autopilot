#include "imu.h"

//#define CALIBRATE_ACCELEROMETER

unsigned long timer;

Imu imu;

void setup() {
  Serial.begin(9600);
  #ifdef CALIBRATE_ACCELEROMETER
  imu.calibrate_accel();
  #else
  imu.calibrate();
  #endif
}

void loop() {
  // timer = micros();
  imu.run();
  // Serial.println(micros() - timer);
  Serial.print(imu.roll());
  Serial.print(" ");
  Serial.print(imu.pitch());
  Serial.print(" ");
  Serial.println(imu.yaw());
}
