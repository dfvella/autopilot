#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>

//#define PRINT_RAW_GYRO
//#define PRINT_RAW_ACCEL
//#define PRINT_IMU_ANGLES
//#define PRINT_PPM_INPUTS
//#define PRINT_MIXED_OUTPUTS
//#define PRINT_PID_OUTPUTS
//#define PRINT_FLIGHT_MODE
//#define PRINT_LOOP_TIME

#ifdef PRINT_RAW_GYRO
#define DO_LOGGING
#define print_raw_gyro() \
    Serial.print("Raw gyro: "); \
    Serial.print(imu.get_raw(Imu::GYROX)); \
    Serial.print(" "); \
    Serial.print(imu.get_raw(Imu::GYROY)); \
    Serial.print(" "); \
    Serial.println(imu.get_raw(Imu::GYROZ));
#else
#define print_raw_gyro()
#endif 

#ifdef PRINT_RAW_ACCEL
#define DO_LOGGING
#define print_raw_accel() \
    Serial.print("Raw accel: "); \
    Serial.print(imu.get_raw(Imu::ACCELX)); \
    Serial.print(" "); \
    Serial.print(imu.get_raw(Imu::ACCELY)); \
    Serial.print(" "); \
    Serial.println(imu.get_raw(Imu::ACCELZ));
#else
#define print_raw_accel()
#endif 

#ifdef PRINT_IMU_ANGLES
#define DO_LOGGING
#define print_imu_angles() \
    Serial.print("Angles: "); \
    Serial.print(imu.roll()); \
    Serial.print(" "); \
    Serial.print(imu.pitch()); \
    Serial.print(" "); \
    Serial.println(imu.yaw());
#else 
#define print_imu_angles() 
#endif

#ifdef PRINT_PPM_INPUTS
#define DO_LOGGING
#define print_ppm_inputs() \
    Serial.print("PPM in: "); \
    Serial.print(ppm.get(ppmDecoder::ARL)); \
    Serial.print(" "); \
    Serial.print(ppm.get(ppmDecoder::ELE)); \
    Serial.print(" "); \
    Serial.print(ppm.get(ppmDecoder::THR)); \
    Serial.print(" "); \
    Serial.print(ppm.get(ppmDecoder::RUD)); \
    Serial.print(" "); \
    Serial.print(ppm.get(ppmDecoder::GER)); \
    Serial.print(" "); \
    Serial.println(ppm.get(ppmDecoder::AUX));
#else 
#define print_ppm_inputs()
#endif

#ifdef PRINT_MIXED_OUTPUTS
#define DO_LOGGING
#define print_mixed_outputs() \
    Serial.print("PWM out: "); \
    Serial.print(servo[RTS]->get()); \
    Serial.print(" "); \
    Serial.print(servo[RBS]->get()); \
    Serial.print(" "); \
    Serial.print(servo[LTS]->get()); \
    Serial.print(" "); \
    Serial.println(servo[LBS]->get());
#else 
#define print_mixed_outputs()
#endif

#ifdef PRINT_PID_OUTPUTS
#define DO_LOGGING
#define print_pid_outputs() \
    Serial.print("PID out: "); \
    Serial.print(servo[RTS]->get()); \
    Serial.print(" "); \
    Serial.println(servo[RBS]->get());
#else 
#define print_pid_outputs()
#endif

#ifdef PRINT_FLIGHT_MODE
#define DO_LOGGING
#define print_flight_mode() \
    Serial.print("flight mode: "); \
    Serial.println(fmode);
#else 
#define print_flight_mode()
#endif

#ifdef PRINT_LOOP_TIME
#define DO_LOGGING
#define print_loop_time() \
    Serial.print("loop: "); \
    Serial.println(loop_time);
#else 
#define print_loop_time()
#endif

#define print_log() \
    print_raw_gyro() \
    print_raw_accel() \
    print_imu_angles() \
    print_ppm_inputs() \
    print_mixed_outputs() \
    print_pid_outputs() \
    print_flight_mode() \
    print_loop_time()

#ifdef DO_LOGGING
#define SERIAL_CONNECTION
#endif

#endif