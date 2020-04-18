#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>

class Servo {

    public:

    Servo(int pin_in) : pin(pin_in) 
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
        timer = micros();
    }

    void low()
    {
        while (micros() - timer < signal);
        digitalWrite(pin, LOW);
    }

    void set(int signal_in)
    {
        signal = signal_in;
    }

    static void write_all(Servo* servo[], const int num)
    {
        Servo* sorted_servo[num];

        for (int i = 0; i < num; ++i)
        {
            sorted_servo[i] = servo[i];
        }
        
        int correct = 0;

        while (correct != num - 1)
        {
            correct = 0;
            for (int i = 0; i + 1 != num; ++i)
            {
                if (sorted_servo[i]->signal <= sorted_servo[i + 1]->signal)
                {
                    ++correct;
                }
                else 
                {
                    Servo* servo_temp = sorted_servo[i];
                    sorted_servo[i] = sorted_servo[i + 1];
                    sorted_servo[i + 1] = servo_temp;
                }
            }
        }

        for (int i = 0; i < num; ++i)
        {
            sorted_servo[i]->high();
        }

        for (int i = 0; i < num; ++i) 
        {
            sorted_servo[i]->low();
        }
    }

    private:

    int pin;

    unsigned long timer;

    int signal;
};

#endif 