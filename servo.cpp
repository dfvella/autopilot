#include "servo.h"

Servo::Servo(int pin_in) : pin(pin_in)
{
    pinMode(pin, OUTPUT);
}

void Servo::set(int signal_in)
{
    if (signal_in > MAX_THROW) 
        signal_in = MAX_THROW;
        
    if (signal_in < MIN_THROW)
        signal_in = MIN_THROW;

    signal = signal_in;
}

int Servo::get()
{
    return signal;
}

void Servo::high()
{
    digitalWrite(pin, HIGH);
    timer = micros();    
}

void Servo::low()
{
    while (micros() - timer < signal);
    digitalWrite(pin, LOW);    
}

void Servo::write(int pulseWidth)
{
    high();
    low();
}

void Servo::write_all(Servo* servo[], const int num)
{
    Servo* sorted_servo[num];

    for (int i = 0; i < num; ++i)
        sorted_servo[i] = servo[i];
    
    int correct = 0;

    while (correct != num - 1)
    {
        correct = 0;
        for (int i = 0; i + 1 != num; ++i)
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

    for (int i = 0; i < num; ++i)
        sorted_servo[i]->high();

    for (int i = 0; i < num; ++i) 
        sorted_servo[i]->low();
}

int map_right_top(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud > 0)
        rud = 0;

    return 3000 - arl - ele - rud + brk;
}

int map_right_bottom(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud > 0)
        rud = 0;

    return arl + ele - rud + brk;
}

int map_left_top(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud < 0)
        rud = 0;

    return 1500 - ((arl - ele + rud) - 1500) - brk;
}

int map_left_bottom(int arl, int ele, int rud, int brk)
{
    ele -= 1500;
    rud -= 1500;

    if (rud < 0)
        rud = 0;

    return arl - ele - rud - brk;
}