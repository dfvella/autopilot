#include "MPU6050DataFetcher.h"

Mpu mpu;

void setup() {
  Serial.begin(9600);
  mpu.begin();
}

void loop() {
  mpu.fetch();
  Serial.println(mpu.get(Mpu6050::ACCELX));
}
