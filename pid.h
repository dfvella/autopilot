#ifndef PID_H
#define PID_H

#include <Arduino.h>

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