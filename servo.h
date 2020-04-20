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
        if (signal_in > 2000) signal_in = 2000;
        if (signal_in < 1000) signal_in = 1000;
        signal = signal_in;
    }

    int get()
    {
        return signal;
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

// args and return val are pwm pulsewidth
int map_right_top(int arl, int ele, int rud)
{
    ele -= 1500;
    rud -= 1500;

    if (rud > 0)
        rud = 0;

    return arl + ele + rud;
}

int map_right_bottom(int arl, int ele, int rud)
{
    ele -= 1500;
    rud -= 1500;

    if (rud > 0)
        rud = 0;

    return arl + ele - rud;
}

int map_left_top(int arl, int ele, int rud)
{
    ele -= 1500;
    rud -= 1500;

    if (rud < 0)
        rud = 0;

    return arl - ele + rud;
}

int map_left_bottom(int arl, int ele, int rud)
{
    ele -= 1500;
    rud -= 1500;

    if (rud < 0)
        rud = 0;

    return arl - ele - rud;
}

#endif 