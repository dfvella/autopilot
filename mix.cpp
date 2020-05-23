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

    return (4 * Servo::CENTER) - arl - ele - rud + brk;
}

uint16_t Mix::right_bottom(uint16_t arl, uint16_t ele, uint16_t rud, uint16_t brk)
{
    if (rud > Servo::CENTER)
        rud = Servo::CENTER;

    return arl + ele - rud + brk;
}

uint16_t Mix::left_top(uint16_t arl, uint16_t ele, uint16_t rud, uint16_t brk)
{
    if (rud < Servo::CENTER)
        rud = Servo::CENTER;

    return Servo::CENTER - ((arl - ele + rud) - Servo::CENTER) - brk;
}

uint16_t Mix::left_bottom(uint16_t arl, uint16_t ele, uint16_t rud, uint16_t brk)
{
    if (rud < Servo::CENTER)
        rud = Servo::CENTER;

    return (2 * Servo::CENTER) + arl - ele - rud - brk;
}