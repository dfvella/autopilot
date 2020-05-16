#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>

class Servo 
{
    public:
        Servo(int pin_in);

        void set(int signal_in);
        int get();

        void write(int pulsewidth);
        static void write_all(Servo* servo[], const int num);

        static constexpr int MAX_THROW = 1800;
        static constexpr int MIN_THROW = 1200;

    private:
        void high();
        void low();

        int pin;
        int signal;

        unsigned long timer;
};

// args and return val are pwm pulsewidth
int map_right_top(int arl, int ele, int rud, int brk);
int map_right_bottom(int arl, int ele, int rud, int brk);
int map_left_top(int arl, int ele, int rud, int brk);
int map_left_bottom(int arl, int ele, int rud, int brk);

#endif 