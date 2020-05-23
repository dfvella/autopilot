/*
 * Class for generating pwm signals to control the servo motors
 * which deflect control surfaces to change the aircraft's attitude
 */

#include "servo.h"

Servo::Servo(uint8_t pin_in) : pin(pin_in)
{
    pinMode(pin, OUTPUT);
}

// Sets the pulsewidth in microseconds of the pulse generated
// by a call to write() or write_all()
void Servo::set(uint16_t signal_in)
{
    if (signal_in > MAX_THROW) 
        signal_in = MAX_THROW;

    if (signal_in < MIN_THROW)
        signal_in = MIN_THROW;

    if (abs(signal_in - signal) > NOISE_FILTER_THRESHOLD)
        signal = signal_in;
}

// Returns the pulsewidth in microseconds of the pulse generated
// by a call to write() or write_all()
uint16_t Servo::get()
{
    return signal;
}

// Writes servo output pin high
void Servo::high()
{
    digitalWrite(pin, HIGH);
    timer = micros();    
}

// Writes servo output pin low
void Servo::low()
{
    while (micros() - timer < signal);
    digitalWrite(pin, LOW);    
}

// Generates a pwm signal on the servo output pin with a pulsewidth
// equal to signal member variable 
void Servo::write(uint16_t signal_in)
{
    high();
    low();
}

// Generates pwm signals on the output pins of all the servos in the 
// array simultaneously 
void Servo::write_all(Servo* servo[], const uint8_t num)
{
    // Make a copt of the servo array
    Servo* sorted_servo[num];

    for (uint8_t i = 0; i < num; ++i)
        sorted_servo[i] = servo[i];
    
    uint8_t correct = 0;

    // Bubble sort the array according to signal pulsewidth
    while (correct != num - 1)
    {
        correct = 0;
        for (uint8_t i = 0; i + 1 != num; ++i)
        {
            if (sorted_servo[i]->signal <= sorted_servo[i + 1]->signal)
                ++correct;
            else 
            {
                Servo* servo_temp = sorted_servo[i];
                sorted_servo[i] = sorted_servo[i + 1];
                sorted_servo[i + 1] = servo_temp;
            }
        }
    }
    
    // Write all the pins high with padding according to PULSE_PADDING
    // between rising edges 
    for (uint8_t i = 0; i < num; ++i)
    {
        sorted_servo[i]->high();
        unsigned long temp = micros();
        while (micros() - temp < PULSE_PADDING);
    }

    // Write all the servo output pins low
    for (uint8_t i = 0; i < num; ++i) 
        sorted_servo[i]->low();
}