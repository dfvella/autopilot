/*
 * Class for generating pwm signals to control the servo motors
 * which deflect control surfaces to change the aircraft's attitude
 */

#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>

// Time between writing output pins high in the write_all() 
// method
#define PULSE_PADDING 20 // microseconds

// Minimum difference required for the signal property to update
// upon a call to set() method. For noise reduction.
#define NOISE_FILTER_THRESHOLD 10 // microseconds


class Servo 
{
    public:
        Servo(uint8_t pin_in);

        void set(uint16_t signal_in);
        uint16_t get();

        void write(uint16_t signal_in);
        static void write_all(Servo* servo[], const uint8_t num);

        static uint16_t limit(uint16_t signal_in);

        static constexpr uint16_t MAX_THROW = 1800;
        static constexpr uint16_t MIN_THROW = 1200;
        static constexpr uint16_t CENTER = 1500;

    private:
        void high();
        void low();

        uint8_t pin;
        uint16_t signal;

        unsigned long timer;
};

#endif 