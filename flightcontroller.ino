#include "imu.h"

Imu imu(13);

void setup() {
  Serial.begin(9600);
  imu.calibrate();
}

void loop() {
  imu.run();
  Serial.print(imu.angle_x());
  Serial.print(" ");
  Serial.println(imu.angle_y());
}
