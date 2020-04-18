#include <Arduino.h>

#include "imu.h"
#include "ppm.h"
#include "servo.h"

#include "logging.h"

//#define CALIBRATE_ACCELEROMETER

unsigned long timer = 0;

const int status_led = 13;

Imu imu(13);
ppmDecoder ppm;

const int THR = 0;
const int ELE = 1;
const int ARL = 2;
const int NUM_SERVOS = 3;
Servo* servo[NUM_SERVOS];

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
  servo[ELE] = new Servo(4);
  servo[ARL] = new Servo(5);
}

void loop() 
{
  ppm.sync();

  if (ppm.error()) digitalWrite(status_led, HIGH);
  else digitalWrite(status_led, LOW);

  servo[THR]->set(ppm.get(ppmDecoder::THR));
  servo[ELE]->set(ppm.get(ppmDecoder::ELE));
  servo[ARL]->set(ppm.get(ppmDecoder::ARL));

  Servo::write_all(servo, NUM_SERVOS);

  imu.run();

  #ifdef DO_LOGGING
  print_log()
  #endif

  while (micros() - timer < 20000);
  timer = micros();
}
