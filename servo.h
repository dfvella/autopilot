#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>

class Servo {
    public:
        Servo(int pin_in) : pin(pin_in) { }

        void begin() 
        {
            pinMode(pin, OUTPUT);
        }
        
        void write(int pulsewidth) 
        {
            digitalWrite(pin, HIGH);
            long unsigned servotimer = micros();
            while (micros() - servotimer < pulsewidth);
            digitalWrite(pin, LOW);
        }

        void high() 
        {
            digitalWrite(pin, HIGH);
        }

        void low()
        {
            digitalWrite(pin, LOW);
        }

    private:
        int pin;
};

#endif 