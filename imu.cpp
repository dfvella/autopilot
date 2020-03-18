#include "imu.h"

Mpu6050::Mpu6050() : data(new int[7]) { }

void Mpu6050::begin() {
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(MPU6050_POWER_MANAGEMENT_REGISTER);
  Wire.write(MPU_6050_POWER_ON);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(MPU6050_ACCELEROMETER_CONFIG_REGISTER);
  Wire.write(ACCELEROMETER_ACCURACY);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(MPU6050_GYRO_CONFIG_REGISTER);
  Wire.write(GYRO_ACCURACY);
  Wire.endTransmission();
}

void Mpu6050::fetch() {
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(MPU6050_ACCEL_XOUT_H_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(MPU6050_I2C_ADDRESS, 14);
  while(Wire.available() < 14);
  
  for (int* ptr = data; ptr < data + 7; ++ptr) {
    *ptr = Wire.read()<<8|Wire.read();
  }
}

int Mpu6050::get(int val) {
  return *(data + val);
}

Mpu6050::~Mpu6050() {
  delete[] data;
}

Imu::Imu(int led_pin) : status_led(led_pin), Mpu6050() { }

void Imu::calibrate() {
  begin();

  pinMode(status_led, OUTPUT);

  long timer;
  int count = 0, rest = 0;
  int x_prev, y_prev, z_prev;
  
  while (rest < PRE_CALIBRATION_REST_TIMER) {
    while (micros() - timer < 4000);
    timer = micros();

    fetch();

    int x_diff = abs(get(ACCELX) - x_prev);
    int y_diff = abs(get(ACCELY) - y_prev);
    int z_diff = abs(get(ACCELZ) - z_prev);

    x_prev = get(ACCELX);
    y_prev = get(ACCELY);
    z_prev = get(ACCELZ);

    if (x_diff < MIN_ACCEL_DIFF 
     && y_diff < MIN_ACCEL_DIFF 
     && z_diff < MIN_ACCEL_DIFF)
      ++rest;
    else rest = 0;

    if (++count % 200 == 0) led_state = !led_state;
    digitalWrite(status_led, led_state);
  }

  x_zero = 0;
  y_zero = 0;
  z_zero = 0;
  
  for (int i = 0; i < GYRO_CALIBRATION_READINGS; ++i) {
    while (micros() - timer < 4000);
    timer = micros();

    fetch();

    x_zero += get(GYROX);
    y_zero += get(GYROY);
    z_zero += get(GYROZ);

    if (++count % 25 == 0) led_state = !led_state;
    digitalWrite(status_led, led_state); 
  }
  x_zero /= GYRO_CALIBRATION_READINGS;
  y_zero /= GYRO_CALIBRATION_READINGS;
  z_zero /= GYRO_CALIBRATION_READINGS;

  x_angle = 0;
  y_angle = 0; // needs to calculate using accel // add acceleration calibration
  z_angle = 0;
}

void Imu::run() {
  fetch();

  x_angle += (get(GYROX) - x_zero) * ANGULAR_RATE_TO_DISPLACEMENT_CONVERSION;
  y_angle += (get(GYROY) - y_zero) * ANGULAR_RATE_TO_DISPLACEMENT_CONVERSION;
  z_angle += (get(GYROZ) - z_zero) * ANGULAR_RATE_TO_DISPLACEMENT_CONVERSION;

  x_angle += y_angle * sin(get(GYROZ) - z_zero) * ANGULAR_RATE_TO_DISPLACEMENT_CONVERSION * DEGREES_TO_RADIANS_CONVERSION;
  y_angle -= x_angle * sin(get(GYROZ) - z_zero) * ANGULAR_RATE_TO_DISPLACEMENT_CONVERSION * DEGREES_TO_RADIANS_CONVERSION;
}

double Imu::angle_x() {
  return x_angle;
}

double Imu::angle_y() {
  return y_angle;
}