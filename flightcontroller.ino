#include <Arduino.h>

#include "imu.h"
#include "ppm.h"
#include "servo.h"
#include "mix.h"
#include "pid.h"

#include "logging.h"

//#define CALIBRATE_ACCELEROMETER
//#define DISABLE_SERVOS

// units: degrees
#define MAX_ROLL_ANGLE 40
#define MAX_PITCH_ANGLE 40

// units microseconds
#define AUTOCLIMB_TRIM 75

// units: degrees
#define PID_ROLL_TRIM 2
#define PID_PITCH_TRIM -1

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

PIDcontroller roll_pid(12, 0, 0, 1);
PIDcontroller pitch_pid(24, 0, 0.5, 1);

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

  servo[THR] = new Servo(4);
  servo[RTS] = new Servo(5);
  servo[RBS] = new Servo(6);
  servo[LTS] = new Servo(7);
  servo[LBS] = new Servo(8);
  
  digitalWrite(status_led, LOW);
  timer = micros();
}

void loop() 
{
  static int state = 0;
  static int fmode = 0;

  const int PASSTHRU = 0;
  const int AUTOLEVEL = 1;
  const int AUTOCLIMB = 2;

  static int arl_out, ele_out, rud_out, brk_out;
  
  imu.run();

  if (state == 0)
  {
    ppm.sync();

    fmode = AUTOCLIMB;
    if (ppm.get(ppmDecoder::AUX) < 1700.0)
      fmode = AUTOLEVEL;
    if (ppm.get(ppmDecoder::AUX) < 1300.0)
      fmode = PASSTHRU;
  }
  if (state == 1)
  {
    double roll_target = (1500.0 - ppm.get(ppmDecoder::ARL)) * 0.04;
    double pitch_target = (1500.0 - ppm.get(ppmDecoder::ELE)) * 0.04;
    if (fmode == PASSTHRU)
    {
      arl_out = ppm.get(ppmDecoder::ARL);
      ele_out = ppm.get(ppmDecoder::ELE);
    }
    else
    {
      arl_out = roll_pid.calculate(imu.roll() - roll_target + PID_ROLL_TRIM) + 1500;
      ele_out = pitch_pid.calculate(imu.pitch() - pitch_target + PID_PITCH_TRIM) + 1500;
    }
    rud_out = ppm.get(ppmDecoder::RUD);

    brk_out = 0;
    if (ppm.get(ppmDecoder::GER) < 1700.0)
      brk_out = 100;
    if (ppm.get(ppmDecoder::GER) < 1300.0)
      brk_out = 400;

  }
  if (state == 2)
  {
    servo[THR]->set(ppm.get(ppmDecoder::THR));

    int trim = 0;
    if (fmode == AUTOCLIMB)
      trim = AUTOCLIMB_TRIM;

    // add definitions for max and min throw for each channel
    servo[RTS]->set(constrain(Mix::right_top(arl_out, ele_out, rud_out, brk_out), 1200, 1800) + trim);
    servo[RBS]->set(constrain(Mix::right_bottom(arl_out, ele_out, rud_out, brk_out), 1200, 1800) - trim);
    servo[LTS]->set(constrain(Mix::left_top(arl_out, ele_out, rud_out, brk_out), 1200, 1800) - trim);
    servo[LBS]->set(constrain(Mix::left_bottom(arl_out, ele_out, rud_out, brk_out), 1200, 1800) + trim);
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
