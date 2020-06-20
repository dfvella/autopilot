#include <Arduino.h>

#include "imu.h"
#include "ppm.h"
#include "servo.h"
#include "mix.h"
#include "pid.h"

#include "logging.h"

//#define CALIBRATE_ACCELEROMETER
#define ENABLE_SERVOS

// units: degrees
#define MAX_ROLL_ANGLE 60.0
#define MAX_PITCH_ANGLE 60.0

// Roll PID gains
#define ROLL_P 8 // was 10,12
#define ROLL_I 0
#define ROLL_D 0.5 // was 0
#define ROLL_I_MAX 1

// Pitch PID gains
#define PITCH_P 4 // was 6, 12
#define PITCH_I 8 // was 0
#define PITCH_D 0.5
#define PITCH_I_MAX 70 // was 1

// units: degrees
#define AUTOCLIMB_TRIM 15 // was 20, 15

// units: degrees
#define PID_ROLL_TRIM 2
#define PID_PITCH_TRIM -1

unsigned long timer = 0;
const int LOOP_TIME = 5000; // microseconds

const uint8_t ppm_pin = 2;
const uint8_t thr_pin = 4;
const uint8_t rts_pin = 5;
const uint8_t rbs_pin = 6;
const uint8_t lts_pin = 7;
const uint8_t lbs_pin = 8;
const uint8_t led_pin = 13;

const uint8_t THR = 0;
const uint8_t RTS = 1;
const uint8_t RBS = 2;
const uint8_t LTS = 3;
const uint8_t LBS = 4;
const uint8_t NUM_SERVOS = 5;
Servo *servo[NUM_SERVOS];

const uint16_t SWITCH_POS1 = 1300;
const uint16_t SWITCH_POS2 = 1700;

Imu imu(led_pin);
ppmDecoder ppm;

PIDcontroller roll_pid(ROLL_P, ROLL_I, ROLL_D, ROLL_I_MAX);
PIDcontroller pitch_pid(PITCH_P, PITCH_I, PITCH_D, PITCH_I_MAX);

void setup() 
{
    pinMode(led_pin, OUTPUT);

    #ifdef SERIAL_CONNECTION
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println("Serial connection");
    #endif

    #ifdef CALIBRATE_ACCELEROMETER
    imu.calibrate_accel();
    #else
    imu.calibrate();
    #endif

    assignPpmDecoderToPin(ppm, ppm_pin);

    servo[THR] = new Servo(thr_pin);
    servo[RTS] = new Servo(rts_pin);
    servo[RBS] = new Servo(rbs_pin);
    servo[LTS] = new Servo(lts_pin);
    servo[LBS] = new Servo(lbs_pin);

    digitalWrite(led_pin, LOW);
    timer = micros();
}

void loop() 
{
    enum State : uint8_t { PPMSYNC, PIDCALC, SERVOSET, SERVOWRITE };
    static State state = PPMSYNC;

    enum FlightMode : uint8_t { PASSTHRU, AUTOLEVEL, AUTOCLIMB };
    static FlightMode fmode;

    static int16_t arl_out, ele_out, rud_out, brk_out;

    imu.run();

    if (state == PPMSYNC)
    {
        ppm.sync();

        fmode = AUTOCLIMB;
        if (ppm.get(ppmDecoder::AUX) < SWITCH_POS2)
            fmode = AUTOLEVEL;
        if (ppm.get(ppmDecoder::AUX) < SWITCH_POS1)
            fmode = PASSTHRU;
    }
    if (state == PIDCALC)
    {
        float roll_target = (float)Servo::CENTER - ppm.get(ppmDecoder::ARL);
        float pitch_target = (float)Servo::CENTER - ppm.get(ppmDecoder::ELE);

        roll_target *= MAX_ROLL_ANGLE / 1000;
        pitch_target *= MAX_PITCH_ANGLE / 1000;

        if (fmode == PASSTHRU)
        {
            arl_out = ppm.get(ppmDecoder::ARL);
            ele_out = ppm.get(ppmDecoder::ELE);
        }
        else
        {
            uint16_t trim = 0;
            if (fmode == AUTOCLIMB)
                trim = AUTOCLIMB_TRIM;

            arl_out = roll_pid.calculate(imu.roll() - roll_target + PID_ROLL_TRIM);
            ele_out = pitch_pid.calculate(imu.pitch() - pitch_target + PID_PITCH_TRIM - trim);

            arl_out = Servo::limit(arl_out + Servo::CENTER);
            ele_out = Servo::limit(ele_out + Servo::CENTER);
        }
        rud_out = ppm.get(ppmDecoder::RUD);

        brk_out = 0;
        if (ppm.get(ppmDecoder::GER) < SWITCH_POS2)
            brk_out = 100;
        if (ppm.get(ppmDecoder::GER) < SWITCH_POS1)
            brk_out = 400;

    }
    if (state == SERVOSET)
    {
        servo[THR]->set(ppm.get(ppmDecoder::THR));

        servo[RTS]->set(Mix::right_top(arl_out, ele_out, rud_out, brk_out));
        servo[RBS]->set(Mix::right_bottom(arl_out, ele_out, rud_out, brk_out));
        servo[LTS]->set(Mix::left_top(arl_out, ele_out, rud_out, brk_out));
        servo[LBS]->set(Mix::left_bottom(arl_out, ele_out, rud_out, brk_out));
    }
    if (state == SERVOWRITE)
    {
        #ifdef ENABLE_SERVOS
        Servo::write_all(servo, NUM_SERVOS);
        #endif 
    }

    if (state == SERVOWRITE) 
        state = PPMSYNC;
    else
        state = (State)(state + 1);

    #ifdef PRINT_LOOP_TIME
    uint16_t loop_time = micros() - timer;
    #endif

    #ifdef DO_LOGGING
    print_log()
    #endif

    if (micros() - timer > LOOP_TIME) 
        digitalWrite(led_pin, HIGH);
    else
        digitalWrite(led_pin, LOW);

    while (micros() - timer < LOOP_TIME);
    timer = micros();
}
