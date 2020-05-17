/*
 * Class for decoding ppm signals sent from the RF receiver
 * using interrupts
 */ 

#include "ppm.h"


ppmDecoder::ppmDecoder() : sync_ptr(data), chan_ptr(data) { }

// Checks if the sync_ptr is pointed to the sync channel in the 
// data array. If not, the data array is traversed until the 
// sync channel is found. MAXIMUM_RESYNC_ATTEMPTS limits the 
// number of complete array traverses.
void ppmDecoder::sync() 
{
    for (uint8_t i = 0; i < MAXIMUM_RESYNC_ATTEMPTS; ++i) 
    {
        if (*sync_ptr > MINIMUM_SYNC_PULSEWIDTH) 
            break;

        if (sync_ptr == data + PPM_DATA_ARRAY_SIZE - 1)
            sync_ptr = data;
        else
            ++sync_ptr;
    }
}

// Returns the pulsewidth of the corresponding receiver channel 
// in milliseconds.
uint16_t ppmDecoder::get(uint8_t chan) 
{
    int val;

    if (sync_ptr + chan <= data + PPM_DATA_ARRAY_SIZE - 1)
        val = sync_ptr[chan];
    else 
        val = sync_ptr[chan - PPM_DATA_ARRAY_SIZE];

    if (val < MINIMUM_PPM_PULSEWIDTH)
        val = MINIMUM_PPM_PULSEWIDTH;

    if (val > MAXIMUM_PPM_PULSEWIDTH)
        val = MAXIMUM_PPM_PULSEWIDTH;

    return val + PPM_PWM_OFFSET;
}

// Updates the data array. Called by interrupt routine
void ppmDecoder::toggle() 
{
    *chan_ptr = micros() - timer;
    timer = micros();

    if (chan_ptr == data + PPM_DATA_ARRAY_SIZE - 1) 
        chan_ptr = data;
    else 
        ++chan_ptr;
}