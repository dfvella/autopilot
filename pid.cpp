/* 
* PID controller class for calculating the servo output signal /
* control surfrace deflection based on current imu readings.
*/

#include "pid.h"
#include "servo.h"

PIDcontroller::PIDcontroller(float p_in, float i_in, float d_in, float i_max_in) 
    : p(p_in), i(i_in), d(d_in), i_max(i_max_in), i_output(0) { }

float PIDcontroller::calculate(float error)
{
    static bool start = true;
    if (start)
    {
        timer = micros();
        start = false;

        return Servo::CENTER;
    }
    else 
    {
        uint16_t t_delta = micros() - timer;
        timer = micros();

        float output = p * error;

        i_output += (t_delta / MICROSEC_PER_SEC) * error;

        if (i_output > i_max) 
            i_output = i_max;

        if (i_output < -1 * i_max) 
            i_output = -1 * i_max;

        output += i * i_output;

        output += d * ((error - prev_error) / t_delta) * MICROSEC_PER_SEC;
        prev_error = error;

        return output;
    }
}