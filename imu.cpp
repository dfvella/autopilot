#include "imu.h"

Mpu6050::Mpu6050() { }

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
  return data[val];
}

Imu::Imu(int led_pin) : status_led(led_pin), Mpu6050(), 
  orientation(Quaternion(1, 0.0001, 0.0001, 0.0001)) { }

void Imu::calibrate() {
  begin();

  pinMode(status_led, OUTPUT);

  int count = 0, rest = 0;
  int x_prev, y_prev, z_prev;
  
  timer = 0;
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

void Imu::calibrate_accel() {
  calibrate();
  Serial.println("Calibrating Accelerometer");

  double x = 0;
  double y = 0;
  double z = 0;
  
  timer = 0;
  int count = 0;
  for (int i = 0; i < ACCEL_CALIBRATION_READINGS; ++i) {
    while (micros() - timer < 4000);
    timer = micros();

    fetch();

    x += get(ACCELX) / (float)ACCEL_CALIBRATION_READINGS;
    y += get(ACCELY) / (float)ACCEL_CALIBRATION_READINGS;
    z += get(ACCELZ) / (float)ACCEL_CALIBRATION_READINGS;

    if (++count % 25 == 0) led_state = !led_state;
    digitalWrite(status_led, led_state); 
  }

  Serial.print(" x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" z: ");
  Serial.print(z);
  Serial.println();
}

void Imu::run() {
  fetch();

  static bool start = true;
  if (start) {
    start = false;
    timer = micros();
  }
  else {
    double t_delta = (micros() - timer) / 1000000.0; // seconds
    timer = micros();

    // create 3D vector of net angular rate
    double w_x = (get(GYROX) - x_zero) * TICKS_PER_DEGREE * DEGREES_TO_RADIANS;
    double w_y = (get(GYROY) - y_zero) * TICKS_PER_DEGREE * DEGREES_TO_RADIANS;
    double w_z = (get(GYROZ) - z_zero) * TICKS_PER_DEGREE * DEGREES_TO_RADIANS;
    Vector w_net(w_x, w_y, w_z);

    // transform angular rate into a unit quaternion representing the rotation
    double w_norm = w_net.norm();
    double r_w = cos((t_delta * w_norm) / 2);
    double r_x = (sin((t_delta * w_norm) / 2) * w_x) / w_norm;
    double r_y = (sin((t_delta * w_norm) / 2) * w_y) / w_norm;
    double r_z = (sin((t_delta * w_norm) / 2) * w_z) / w_norm;
    Quaternion rotation(r_w, r_x, r_y, r_z);

    // apply rotation
    orientation = Quaternion::product(orientation, rotation);

    // normalize, noise reduction
    orientation.normalize();

    // calculate roll pitch and yaw
    pitch = orientation.pitch() / DEGREES_TO_RADIANS;
    roll = orientation.roll() / DEGREES_TO_RADIANS;
    yaw = orientation.yaw() / DEGREES_TO_RADIANS;

    // x_angle += (get(GYROX) - x_zero) * ANGULAR_RATE_TO_DISPLACEMENT_CONVERSION * t_delta;
    // y_angle += (get(GYROY) - y_zero) * ANGULAR_RATE_TO_DISPLACEMENT_CONVERSION * t_delta;
    // z_angle += (get(GYROZ) - z_zero) * ANGULAR_RATE_TO_DISPLACEMENT_CONVERSION * t_delta;

    // x_angle += y_angle * sin((get(GYROZ) - z_zero) * ANGULAR_RATE_TO_DISPLACEMENT_CONVERSION * t_delta * DEGREES_TO_RADIANS_CONVERSION);
    // y_angle -= x_angle * sin((get(GYROZ) - z_zero) * ANGULAR_RATE_TO_DISPLACEMENT_CONVERSION * t_delta * DEGREES_TO_RADIANS_CONVERSION);
  }
}

// double Imu::angle_x() {
//   return angle_x;
// }

// double Imu::angle_y() {
//   return angle_y;
// }

double Imu::get_roll() {
  return roll;
}

double Imu::get_pitch() {
  return pitch;
}

double Imu::get_yaw() {
  return yaw;
}

Imu::Quaternion::Quaternion(double w, double x, double y, double z) 
  : w(w), x(x), y(y), z(z) { }

Imu::Quaternion Imu::Quaternion::product(Quaternion p, Quaternion q) {
  double w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z;
  double x = p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y;
  double y = p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x;
  double z = p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w;
  
  Quaternion result(w, x, y, z);

  return result;
}

double Imu::Quaternion::norm() {
  return sqrt(w * w + x * x + y * y + z * z);
}

void Imu::Quaternion::normalize() {
  double l = norm();
  w /= l;
  x /= l;
  y /= l;
  x /= l;
}

void Imu::Quaternion::conjugate() {
  x *= -1;
  y *= -1;
  z *= -1;
}

double Imu::Quaternion::pitch() {
  return atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z);
}

double Imu::Quaternion::roll() {
  return atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z);
}

double Imu::Quaternion::yaw() {
  return asin(2 * x * y + 2 * z * w);
}

Imu::Vector::Vector(double x, double y, double z)
  : x(x), y(y), z(z) { }

double Imu::Vector::norm() {
  return sqrt(x * x + y * y + z * z);
}

void Imu::Vector::rotate(Quaternion q) {
  Quaternion r(0, x, y, z);
  
  Quaternion q_i = q;
  q_i.conjugate();
  
  Quaternion result = Quaternion::product(Quaternion::product(q, r), q_i);
  
  x = result.x;
  y = result.y;
  z = result.z;
}