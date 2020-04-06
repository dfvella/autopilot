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

#define GRAVITY_REFERENCED_ZERO

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


class Imu {
  public:
    // constructor sets the intial orientation of the aircaft
    Imu();

    // Waits until the accelermeter does not detect significant motion.
    // Takes calibration readings to determine the gyroscope raw readings
    // while at rest. Then, zeroes the gyroscope with reference to the
    // gravitational force vector caclulated from the accelermeter data. 
    void calibrate();

    // Imediatly begins sampling accelermeter data and prints the average
    // of these readings to the serial monitor. Mpu6050 should be placed
    // on a level surface.
    void calibrate_accel();

    // Samples the MPU6050 data. Constructs a 3D vector representing the
    // angular velcoity of the aircraft. Then, transforms this vector into
    // a unit quaternion representing the rotation. Computes the product of 
    // the last orientation and the rotation to determine the new orientation
    // of the aircaft.
    void run();

    // Returns the roll angle of the aircaft in degrees
    //  -180 < roll < 180
    double roll();

    // Returns the pitch angle of the aircaft in degrees
    //  -90 < pitch < 90
    double pitch();

    // Returns the yaw angle of the aircaft in degress
    //  -180 < yaw < 180
    double yaw();

  private:
    static constexpr double RADIANS_PER_DEGREE = 0.01745329;

    struct Quaternion {
        double w, x, y, z;
    };

    // computes then returns the product of two quaternions
    Quaternion product(const Quaternion &p, const Quaternion &q);

    // computes then returns the norm/length of a quaternion
    double norm(const Quaternion &q);

    struct Vector {
      double x, y, z;
    };

    // computes then returns the norm/length of a quaternion
    double norm(const Vector &v);

    // stores the aircaft's current orientation
    Quaternion orientation;

    // stores the aircaft's current roll, pitch, and yaw angles
    double x_angle, y_angle, z_angle;
    
    // stores the gyroscope's raw angular velcoity readings while not moving
    double x_zero, y_zero, z_zero;

    // stores the system clock time when orientation was last determined
    unsigned long timer;

    // excapsulates the I2C interface with the Mpu6050 sensor
    class Mpu6050 {
      public:
        Mpu6050();

        // Begins I2C communication with the Mpu6050 sensor. Configures the
        // Mpu6050 power settings, gyroscope accuracy, and accelermeter
        // accuracy.
        void begin();

        // Opens I2C communication with the Mpu6050 sensor. Retreieves raw
        // gyroscope, accelermeter, and temperature data and stores it in the 
        // private int array data.
        void fetch();

        // returns the requested raw Mpu6050 data
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

    Mpu6050 mpu;

    int status_led = 13;
    bool led_state;
};

#endif
