#include "pid.h"

PIDcontroller::PIDcontroller(double p_in, double i_in, double d_in, double i_max_in) 
    : p(p_in), i(i_in), d(d_in), i_max(i_max_in) { }

double PIDcontroller::calculate(double error)
{
    static bool start = true;
    if (start)
    {
      timer = micros();
      start = false;
      return 1500;
    }
    else 
    {
      int t_delta = micros() - timer;
      timer = micros();

      double output = p * error;

      i_output += t_delta * error;
      if (i_output > i_max) i_output = i_max;
      if (i_output < -1 * i_max) i_output = -1 * i_max;
      output += i * i_output;

      output += d * ((error - prev_error) / t_delta) * 1000000;
      prev_error = error;

      return output;
    }
}