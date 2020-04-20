#include <Arduino.h>

#include "imu.h"
#include "ppm.h"
#include "servo.h"

#include "logging.h"

//#define CALIBRATE_ACCELEROMETER
//#define DISABLE_SERVOS

#define MAX_ROLL_ANGLE 40
#define MAX_PITCH_ANGLE 40

unsigned long timer = 0;

const int status_led = 13;

Imu imu(status_led);
ppmDecoder ppm;

const int THR = 0;
const int RTS = 1;
const int RBS = 2;
const int LTS = 3;
const int LBS = 4;
const int NUM_SERVOS = 5;
Servo* servo[NUM_SERVOS];

class PIDcontroller
{
  public:
  PIDcontroller(int p_in, int i_in, int d_in, int i_max_in) 
    : p(p_in), i(i_in), d(d_in), i_max(i_max_in) { }
  double calculate(double error)
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

      output += d * ((error - prev_error) / t_delta);
      error = prev_error;
      return output;
    }  
  }
  private:
  int p;
  int i;
  int d;
  int i_max;
  double i_output;
  unsigned long timer;
  double prev_output;
  double prev_error;
};

PIDcontroller roll_pid(12, 0, 0, 1);
PIDcontroller pitch_pid(12, 0, 0, 1);

void setup() 
{
  pinMode(status_led, OUTPUT);

  #ifdef SERIAL_CONNECTION
  Serial.begin(9600);
  Serial.println("Serial connection");
  #endif

  #ifdef CALIBRATE_ACCELEROMETER
  imu.calibrate_accel();
  #else
  imu.calibrate();
  #endif

  assignPpmDecoderToPin(ppm, 2);

  servo[THR] = new Servo(3);
  servo[RTS] = new Servo(4);
  servo[RBS] = new Servo(5);
  servo[LTS] = new Servo(6);
  servo[LBS] = new Servo(7);
  
  digitalWrite(status_led, LOW);
  timer = micros();
}

void loop() 
{
  static int state = 0;

  static int fmode = 0;
  const int DISARMED = 0;
  const int PASSTHRU = 1;
  const int AUTOLEVEL = 2;

  static int arl_out, ele_out, rud_out;
  
  imu.run();

  if (state == 0)
  {
    ppm.sync();

    fmode = AUTOLEVEL;
    if (ppm.get(ppmDecoder::AUX) < 1700)
      fmode = DISARMED;
    if (ppm.get(ppmDecoder::AUX) < 1300)
      fmode = PASSTHRU;
  }
  if (state == 1)
  {
    double roll_target = (ppm.get(ppmDecoder::ARL) - 1500) * 0.04;
    double pitch_target = (ppm.get(ppmDecoder::ELE) - 1500) * 0.04;
    if (fmode == PASSTHRU)
    {
      arl_out = ppm.get(ppmDecoder::ARL);
      ele_out = ppm.get(ppmDecoder::ELE);
    }
    else
    {
      arl_out = roll_pid.calculate(roll_target - imu.roll()) + 1500;
      ele_out = pitch_pid.calculate(pitch_target - imu.pitch()) + 1500;
    }
    rud_out = ppm.get(ppmDecoder::RUD);
  }
  if (state == 2)
  {
    if (fmode == DISARMED)
      servo[THR]->set(1000);
    else
      servo[THR]->set(ppm.get(ppmDecoder::THR));

    servo[RTS]->set(map_right_top(arl_out, ele_out, rud_out));
    servo[RBS]->set(map_right_bottom(arl_out, ele_out, rud_out));
    servo[LTS]->set(map_left_top(arl_out, ele_out, rud_out));
    servo[LBS]->set(map_left_bottom(arl_out, ele_out, rud_out));
  }
  if (state == 3)
  {
    #ifndef DISABLE_SERVOS
    Servo::write_all(servo, NUM_SERVOS);
    #endif 
  }

  if (state == 3) 
    state = 0;
  else
    ++state;

  #ifdef PRINT_LOOP_TIME
  int loop_time = micros() - timer;
  #endif

  #ifdef DO_LOGGING
  print_log()
  #endif

  if (micros() - timer > 5000) digitalWrite(status_led, HIGH);
  while (micros() - timer < 5000);
  timer = micros();
}
