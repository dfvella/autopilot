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
  orientation(Quaternion(0.7071, 0.7071, 0.0001, 0.0001)) { }

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

  double accel_x = 0;
  double accel_z = 0;
  double accel_y = 0;
  
  for (int i = 0; i < GYRO_CALIBRATION_READINGS; ++i) {
    while (micros() - timer < 4000);
    timer = micros();

    fetch();

    x_zero += get(GYROX);
    y_zero += get(GYROY);
    z_zero += get(GYROZ);

    accel_x += get(ACCELX);
    accel_y += get(ACCELY);
    accel_z += get(ACCELZ);

    if (++count % 25 == 0) led_state = !led_state;
    digitalWrite(status_led, led_state); 
  }
  x_zero /= GYRO_CALIBRATION_READINGS;
  y_zero /= GYRO_CALIBRATION_READINGS;
  z_zero /= GYRO_CALIBRATION_READINGS;

  accel_x /= GYRO_CALIBRATION_READINGS;
  accel_y /= GYRO_CALIBRATION_READINGS;
  accel_z /= GYRO_CALIBRATION_READINGS;

  accel_x -= ACCELX_LEVEL_READING;
  accel_y -= ACCELY_LEVEL_READING;
  accel_z -= ACCELZ_LEVEL_READING;

  #ifdef GRAVITY_ZERO
  Vector net_accel = { accel_x, accel_y, accel_z };
  double accel_angle_x = asin(accel_y / norm(net_accel)) * (1 / RADIANS_PER_DEGREE);
  double accel_angle_y = asin(accel_x / norm(net_accel)) * (1 / RADIANS_PER_DEGREE) * -1;

  Quaternion initial_roll(cos(accel_angle_x * RADIANS_PER_DEGREE * 0.5), 
                          sin(accel_angle_x * RADIANS_PER_DEGREE * 0.5), 0.0001, 0.0001);

  orientation = Quaternion::product(orientation, initial_roll);
  orientation.normalize();

  Quaternion initial_pitch(cos(accel_angle_y * RADIANS_PER_DEGREE * 0.5), 0.0001, 
                            sin(accel_angle_y * RADIANS_PER_DEGREE * 0.5), 0.0001);

  orientation = Quaternion::product(orientation, initial_pitch);
  orientation.normalize();
  #endif
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
  Serial.print(z - TICKS_PER_G);
  Serial.println();
}

void Imu::run() {
  fetch();

  static bool start = true;
  if (start) {
    start = false;
    timer = micros();

    w_x_prev = get(GYROX);
    w_y_prev = get(GYROY);
    w_z_prev = get(GYROZ);
  }
  else {
    double t_delta = (micros() - timer) / 1000000.0; // seconds
    timer = micros();

    // create 3D vector of net angular rate
    double w_x = (((get(GYROX) + w_x_prev) / 2) - x_zero) * TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
    double w_y = (((get(GYROY) + w_y_prev) / 2) - y_zero) * TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
    double w_z = (((get(GYROZ) + w_z_prev) / 2) - z_zero) * TICKS_PER_DEGREE * RADIANS_PER_DEGREE;
    Vector w_net = { w_x, w_y, w_z };

    // save the current gyro data as the previous data for next iteration
    w_x_prev = get(GYROX);
    w_y_prev = get(GYROY);
    w_z_prev = get(GYROZ);

    // transform angular rate into a unit quaternion representing the rotation
    double w_norm = norm(w_net);
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
    pitch = orientation.pitch();
    roll = orientation.roll();
    yaw = orientation.yaw();
  }
}

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

double Imu::Quaternion::roll() {
  double result = atan2(2 * x * w - 2 * y * z, 1 - 2 * x * x - 2 * z * z) 
                                                * -1 / RADIANS_PER_DEGREE + 90;
  return result > 180 ? result - 360 : result;
}

double Imu::Quaternion::pitch() {
  return asin(2 * x * y + 2 * z * w) / RADIANS_PER_DEGREE;
}

double Imu::Quaternion::yaw() {
  return atan2(2 * y * w - 2 * x * z, 1 - 2 * y * y - 2 * z * z) / RADIANS_PER_DEGREE;
}

double Imu::norm(const Vector &v) {
  return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}