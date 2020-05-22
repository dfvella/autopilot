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

class PIDcontroller
{
    public:
        PIDcontroller(double p_in, double i_in, double d_in, double i_max_in);
        double calculate(double error);

    private:
        double p, i, d, i_max;
        double i_output, prev_output, prev_error;

        unsigned long timer;
};

#endif 