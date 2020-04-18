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

  servo[THR]->begin();
  servo[ELE]->begin(); // move into ctor?
  servo[ARL]->begin();
}

void loop() 
{
  ppm.sync();

  if (ppm.error()) digitalWrite(status_led, HIGH);
  else digitalWrite(status_led, LOW);

  int sorted_signal[NUM_SERVOS];
  sorted_signal[THR] = ppm.get(ppmDecoder::THR);
  sorted_signal[ELE] = ppm.get(ppmDecoder::ELE);
  sorted_signal[ARL] = ppm.get(ppmDecoder::ARL);

  Servo* sorted_servo[NUM_SERVOS];
  sorted_servo[THR] = servo[THR];
  sorted_servo[ELE] = servo[ELE];
  sorted_servo[ARL] = servo[ARL];

  int correct = 0;
  while (correct != NUM_SERVOS - 1)
  {
    correct = 0;
    for (int i = 0; i + 1 != NUM_SERVOS; ++i)
    {
      if (sorted_signal[i] <= sorted_signal[i + 1])
      {
        ++correct;
      }
      else 
      {
        Servo* servo_temp = sorted_servo[i];
        sorted_servo[i] = sorted_servo[i + 1];
        sorted_servo[i + 1] = servo_temp;
        int signal_temp = sorted_signal[i];
        sorted_signal[i] = sorted_signal[i + 1];
        sorted_signal[i + 1] = signal_temp;
      }
    }
  }
  
  unsigned long servo_timer[NUM_SERVOS];

  sorted_servo[0]->high();
  servo_timer[0] = micros();

  delayMicroseconds(20);

  sorted_servo[1]->high();
  servo_timer[1] = micros();
  
  delayMicroseconds(20);

  sorted_servo[2]->high();
  servo_timer[2] = micros();

  while(micros() - servo_timer[0] < sorted_signal[0]);
  sorted_servo[0]->low();

  while(micros() - servo_timer[1] < sorted_signal[1]);
  sorted_servo[1]->low();

  while(micros() - servo_timer[2] < sorted_signal[2]);
  sorted_servo[2]->low();

  // Serial.print("Sorted: ");
  // Serial.print(sorted_signal[0]);
  // Serial.print(" ");
  // Serial.print(sorted_signal[1]);
  // Serial.print(" ");
  // Serial.println(sorted_signal[2]);

  // servo[0]->write(ppm.get(ppmDecoder::THR));
  // servo[1]->write(ppm.get(ppmDecoder::ELE));
  // servo[2]->write(ppm.get(ppmDecoder::ARL));

  imu.run();

  #ifdef DO_LOGGING
  print_log()
  #endif

  while (micros() - timer < 20000);
  timer = micros();
}
