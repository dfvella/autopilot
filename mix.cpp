/*
 * Functions that convert the raw aileron, elevator, rudder, and brake
 * inputs and convert them to outputs to the split elevons 
 */

#include "mix.h"
#include "servo.h"

uint16_t Mix::right_top(uint16_t arl, uint16_t ele, uint16_t rud, uint16_t brk)
{
    if (rud > Servo::CENTER)
        rud = Servo::CENTER;

    uint16_t output = (4 * Servo::CENTER) - arl - ele - rud + brk;

    return Servo::limit(output);
}

uint16_t Mix::right_bottom(uint16_t arl, uint16_t ele, uint16_t rud, uint16_t brk)
{
    if (rud > Servo::CENTER)
        rud = Servo::CENTER;

    uint16_t output = arl + ele - rud + brk;

    return Servo::limit(output);
}

uint16_t Mix::left_top(uint16_t arl, uint16_t ele, uint16_t rud, uint16_t brk)
{
    if (rud < Servo::CENTER)
        rud = Servo::CENTER;

    uint16_t output = Servo::CENTER - ((arl - ele + rud) - Servo::CENTER) - brk;

    return Servo::limit(output);
}

uint16_t Mix::left_bottom(uint16_t arl, uint16_t ele, uint16_t rud, uint16_t brk)
{
    if (rud < Servo::CENTER)
        rud = Servo::CENTER;

    uint16_t output = (2 * Servo::CENTER) + arl - ele - rud - brk;

    return Servo::limit(output);
}