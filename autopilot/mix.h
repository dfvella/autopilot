/*
 * Functions that convert the raw aileron, elevator, rudder, and brake
 * inputs and convert them to outputs to the split elevons 
 */

#ifndef MIX_H
#define MIX_H

#include <Arduino.h>

namespace Mix 
{
    uint16_t right_top(uint16_t arl, uint16_t ele, uint16_t rud, uint16_t brk);
    uint16_t right_bottom(uint16_t arl, uint16_t ele, uint16_t rud, uint16_t brk);
    uint16_t left_top(uint16_t arl, uint16_t ele, uint16_t rud, uint16_t brk);
    uint16_t left_bottom(uint16_t arl, uint16_t ele, uint16_t rud, uint16_t brk);
}

#endif