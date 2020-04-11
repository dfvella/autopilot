#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>

//#define PRINT_IMU_ANGLES
#define PRINT_PPM_INPUTS
//#define PRINT_LOOP_TIME

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

#ifdef PRINT_LOOP_TIME
#define DO_LOGGING
#define print_loop_time() \
    Serial.print("loop time: "); \
    Serial.println(timer);
#else 
#define print_loop_time()
#endif

#define print_log() \
    print_imu_angles() \
    print_ppm_inputs() \
    print_loop_time()

#ifdef DO_LOGGING
#define SERIAL_CONNECTION
#endif

#endif