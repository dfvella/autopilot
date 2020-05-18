/*
 * Class for decoding ppm signals sent from the RF receiver
 * using interrupts
 */ 

#ifndef PPM_H
#define PPM_H

#include <Arduino.h>

#define MAXIMUM_PPM_PULSEWIDTH 1700
#define MINIMUM_PPM_PULSEWIDTH 700
#define MINIMUM_SYNC_PULSEWIDTH 10000
#define PPM_PWM_OFFSET 300
#define MAXIMUM_RESYNC_ATTEMPTS 5
#define RECEIVER_CHANNELS 6
#define PPM_DATA_ARRAY_SIZE (RECEIVER_CHANNELS + 1) * 2


class ppmDecoder 
{
    public:
        ppmDecoder();

        // Checks if the sync_ptr is pointed to the sync channel in the 
        // data array. If not, the data array is traversed until the 
        // sync channel is found. MAXIMUM_RESYNC_ATTEMPTS limits the 
        // number of complete array traverses.
        void sync();

        // Returns the pulsewidth of the corresponding receiver channel 
        // in milliseconds.
        int16_t get(uint8_t chan);

        // Receiver channels used to index the data array
        static constexpr uint8_t ARL = 2;
        static constexpr uint8_t ELE = 4;
        static constexpr uint8_t THR = 6;
        static constexpr uint8_t RUD = 8;
        static constexpr uint8_t GER = 10;
        static constexpr uint8_t AUX = 12;

        // Updates the data array. Called by interrupt routine
        void toggle();

    private:
        uint16_t data[PPM_DATA_ARRAY_SIZE];

        uint16_t* chan_ptr;
        uint16_t* sync_ptr;

        unsigned long timer;
};

// Associates an instance of the ppmDecoder class with a pin.
// Macro defined so compiler can deduce instance of ppmDecoder class
#define assignPpmDecoderToPin(handler, pin) \
    pinMode(pin, INPUT_PULLUP); \
    attachInterrupt( \
        digitalPinToInterrupt(pin), \
        [](void){ handler.toggle(); }, \
        CHANGE \
    );

// Disassociates an instance of the ppmDecoder class with a pin
#define removePpmDecoderFromPin(handler, pin) \
    detachInterrupt(pin);

#endif