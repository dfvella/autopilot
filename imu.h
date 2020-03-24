#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

// IMU CALIBRATION SETTINGS
#define PRE_CALIBRATION_REST_TIMER 500
#define MIN_ACCEL_DIFF 200
#define GYRO_CALIBRATION_READINGS 1000
#define ACCEL_CALIBRATION_READINGS 5000

#define ACCELX_LEVEL_READING 485
#define ACCELY_LEVEL_READING -138
#define ACCELZ_LEVEL_READING -3081

#define ANGULAR_RATE_TO_DISPLACEMENT_CONVERSION 0.0152671756 // 1 / 65.5
#define DEGREES_TO_RADIANS_CONVERSION 0.01745329 

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

    const static int ACCELX = 0;
    const static int ACCELY = 1;
    const static int ACCELZ = 2;
    const static int TEMP = 3;
    const static int GYROX = 4;
    const static int GYROY = 5;
    const static int GYROZ = 6;

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

  private:
    double x_angle, y_angle, z_angle;
    double x_angle_prev, y_angle_prev, z_angle_prev;
    
    double x_zero, y_zero, z_zero;

    unsigned long timer;

    int status_led;
    bool led_state;
};

#endif
