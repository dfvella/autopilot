#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

// IMU CALIBRATION SETTINGS
#define PRE_CALIBRATION_REST_TIMER 500
#define MIN_ACCEL_DIFF 200
#define GYRO_CALIBRATION_READINGS 1000
#define ACCEL_CALIBRATION_READINGS 5000

#define ACCELX_LEVEL_READING 139
#define ACCELY_LEVEL_READING -160
#define ACCELZ_LEVEL_READING 1157

#define GRAVITY_ZERO

#define INVERT_ROLL_AXIS
//#define INVERT_PITCH_AXIS
//#define INVERT_YAW_AXIS

//MPU6050 ADDRESSES
#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_POWER_MANAGEMENT_REGISTER 0x6B 
#define MPU6050_GYRO_CONFIG_REGISTER 0x1B
#define MPU6050_ACCELEROMETER_CONFIG_REGISTER 0x1C
#define MPU6050_ACCEL_XOUT_H_REGISTER 0x3B

//MPU6050 POWER CONFIG
#define MPU_6050_POWER_ON 0b00000000

//GYRO CONFIG
//#define GYRO_ACCURACY 0b00000000       // +/-  250 deg/sec
#define GYRO_ACCURACY 0b00001000       // +/-  500 deg/sec
//#define GYRO_ACCURACY 0b00010000       // +/- 1000 deg/sec
//#define GYRO_ACCURACY 0b00011000       // +/- 2000 deg/sec

//ACCELEROMETER CONFIG
//#define ACCELEROMETER_ACCURACY 0b00000000       // +/-  2 g's
//#define ACCELEROMETER_ACCURACY 0b00001000       // +/-  4 g's
#define ACCELEROMETER_ACCURACY 0b00010000       // +/-  8 g's
//#define ACCELEROMETER_ACCURACY 0b00011000       // +/- 16 g's


class Mpu6050 {
  public:
    Mpu6050();
    void begin();
    void fetch();
    int get(int val);

    static constexpr int ACCELX = 0;
    static constexpr int ACCELY = 1;
    static constexpr int ACCELZ = 2;
    static constexpr int TEMP = 3;
    static constexpr int GYROX = 4;
    static constexpr int GYROY = 5;
    static constexpr int GYROZ = 6;

    static constexpr double TICKS_PER_DEGREE = 0.0152671756;
    static constexpr double TICKS_PER_G = 4096.0;

  private:
    int data[7];
};

class Imu : public Mpu6050 {
  public:
    Imu(int led_pin);
    void calibrate();
    void calibrate_accel(); // place on level surface
    void run();
    double angle_x();
    double angle_y();

    double roll();
    double pitch();
    double yaw();

    static constexpr double RADIANS_PER_DEGREE = 0.01745329;

  private:
    struct Quaternion {
        double w, x, y, z;
    };

    Quaternion product(const Quaternion &p, const Quaternion &q);
    double norm(const Quaternion &q);
    void normalize(Quaternion &q); // move into product?

    struct Vector {
      double x, y, z;
    };

    double norm(const Vector &v);

    Quaternion orientation;
    double x_angle, y_angle, z_angle;
    
    double x_zero, y_zero, z_zero;

    unsigned long timer;

    int status_led;
    bool led_state;
};

#endif
