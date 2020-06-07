#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

// ********** LED INDICATOR CODES **********
// slow blink - waiting for aircraft to be stationary
// fast blink - calibrating (do not move)

// ********** ACCELEROMETER CALIBRATION **********
#define ACCELX_LEVEL_READING -9
#define ACCELY_LEVEL_READING -44
#define ACCELZ_LEVEL_READING 1125

// ********** FILTER GAINS **********
#define ACCEL_FILTER_GAIN 0.1
#define DRIFT_FILTER_GAIN 0.98
//#define ENABLE_GYRO_DRIFT_FILTER 
//NOTE: increases run() runtime by 10-15%

// ********** INVERT AXES **********
#define INVERT_ROLL_AXIS
//#define INVERT_PITCH_AXIS
//#define INVERT_YAW_AXIS

// ********** IMU CALIBRATION ROUTINE SETTINGS **********
#define PRE_CALIBRATION_REST_TIMER 1500
#define MIN_ACCEL_DIFF 200
#define GYRO_CALIBRATION_READINGS 1000
#define ACCEL_CALIBRATION_READINGS 5000
// When enabled, the aircraft does not have to be level during calibration.
// Instead, the net acceleration vector will be used to determine initial
// orientation.
#define ENABLE_GRAVITY_REFERENCED_ZERO



// ********** MPU6050 I2C REGISTER ADDRESSES **********
#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_POWER_MANAGEMENT_REGISTER 0x6B 
#define MPU6050_GYRO_CONFIG_REGISTER 0x1B
#define MPU6050_ACCELEROMETER_CONFIG_REGISTER 0x1C
#define MPU6050_ACCEL_XOUT_H_REGISTER 0x3B

// ********** MPU6050 POWER SETTINGS **********
#define MPU_6050_POWER_ON 0b00000000

// ********** MPU6050 GYROSCOPE ACCURACY SETTINGS **********
//#define GYRO_ACCURACY 0b00000000       // +/-  250 deg/sec
#define GYRO_ACCURACY 0b00001000       // +/-  500 deg/sec
//#define GYRO_ACCURACY 0b00010000       // +/- 1000 deg/sec
//#define GYRO_ACCURACY 0b00011000       // +/- 2000 deg/sec

// ********** MPU6050 ACCELEROMETER ACCURACY SETTINGS **********
//#define ACCELEROMETER_ACCURACY 0b00000000       // +/-  2 g's
//#define ACCELEROMETER_ACCURACY 0b00001000       // +/-  4 g's
#define ACCELEROMETER_ACCURACY 0b00010000       // +/-  8 g's
//#define ACCELEROMETER_ACCURACY 0b00011000       // +/- 16 g's



class Imu 
{
    public:
        // constructor sets the initial orientation of the aircraft
        // OPTIONAL: provide a pin number for a status led indicator
        Imu(int8_t pin);

        // default constructor disabled status led indicator
        Imu();

        // Waits until the accelerometer does not detect significant motion.
        // Takes calibration readings to determine the gyroscope raw readings
        // while at rest. Then, zeroes the gyroscope with reference to the
        // gravitational force vector calculated from the accelerometer data. 
        void calibrate();

        // Immediately begins sampling accelerometer data and prints the average
        // of these readings to the serial monitor. Mpu6050 should be placed
        // on a level surface.
        // NOTE: Serial.begin() must be called before using this function
        void calibrate_accel();

        // Samples the MPU6050 data. Constructs a 3D vector representing the
        // angular velocity of the aircraft. Then, transforms this vector into
        // a unit quaternion representing the rotation. Computes the product of 
        // the last orientation and the rotation to determine the new orientation
        // of the aircraft.
        // NOTE: this function should be called at frequencies between 10-250hz for
        // accurate angle calculations
        void run();

        // Returns the roll angle of the aircraft in degrees
        //  -180 < roll < 180
        float roll();

        // Returns the pitch angle of the aircraft in degrees
        //  -90 < pitch < 90
        float pitch();

        // Returns the yaw angle of the aircraft in degrees
        //  -180 < yaw < 180
        float yaw();

        // Returns 4 byte int pulled directly from MPU6050 registers
        int32_t get_raw(uint8_t val);

        // Use as args for get_raw() and MPU6050::get()
        // Indexes for MPU6050 data array
        static constexpr uint8_t ACCELX = 0;
        static constexpr uint8_t ACCELY = 1;
        static constexpr uint8_t ACCELZ = 2;
        static constexpr uint8_t TEMP = 3;
        static constexpr uint8_t GYROX = 4;
        static constexpr uint8_t GYROY = 5;
        static constexpr uint8_t GYROZ = 6;

    private:
        static constexpr float RADIANS_PER_DEGREE = 0.01745329;

        struct Quaternion 
        {
            float w, x, y, z;
        };

        // computes then returns the product of two quaternions
        Quaternion product(const Quaternion &p, const Quaternion &q);

        // computes then returns the norm/length of a quaternion
        float norm(const Quaternion &q);

        struct Vector 
        {
            float x, y, z;
        };

        // computes then returns the norm/length of a quaternion
        float norm(const Vector &v);

        // stores the aircraft's current orientation
        Quaternion orientation;

        // stores the aircraft's current roll, pitch, and yaw angles in degrees
        float x_angle, y_angle, z_angle;

        // stores the aircraft's current roll and pitch angles in degrees
        // calculated only using the net acceleration on the aircraft
        float x_angle_accel, y_angle_accel;

        // stores the gyroscope's raw angular velocity readings while not moving
        float x_zero, y_zero, z_zero;

        // stores the system clock time when orientation was last determined
        unsigned long timer;

        // manages I2C interface with the Mpu6050 sensor
        class Mpu6050 
        {
            public:
                Mpu6050();

                // Begins I2C communication with the Mpu6050 sensor. Configures the
                // Mpu6050 power settings, gyroscope accuracy, and accelerometer
                // accuracy.
                void begin();

                // Opens I2C communication with the Mpu6050 sensor. Retrieves raw
                // gyroscope, accelerometer, and temperature data and stores it in the 
                // private int array data.
                void fetch();

                // returns the requested raw Mpu6050 data
                int32_t get(uint8_t val);

                static constexpr float TICKS_PER_DEGREE = 0.0152672;
                static constexpr float TICKS_PER_G = 4096.0;

            private:
                // Stores the raw data Mpu6050 sensor data retrieved from the most 
                // recent fetch() call.
                int32_t data[7];
        };

        Mpu6050 mpu;

        uint8_t status_led;
};

#endif // IMU_H
