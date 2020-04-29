#ifndef PPM_H
#define PPM_H

#include <Arduino.h>

#define MAXIMUM_PPM_PULSEWIDTH 2000
#define MINIMUM_PPM_PUSLEWIDTH 1000

#define MIN_SYNC_PULSEWIDTH 10000

#define MAXIMUM_RESYNC_ATTEMPTS 5

// #define PPM_PRINT_WARNINGS
//#define TELEMETRY_CHECKING

class ppmDecoder {
    public:
        ppmDecoder();
        void sync();
        int get(int chan);
        bool error();
        void toggle();

        static constexpr int ARL = 2;
        static constexpr int ELE = 4;
        static constexpr int THR = 6;
        static constexpr int RUD = 8;
        static constexpr int GER = 10;
        static constexpr int AUX = 12;

    private:
        bool is_synced;
        bool is_connected;

        unsigned long timer;
        int data[14];
        int* chan_ptr;
        int* sync_ptr;
};

#define assignPpmDecoderToPin(handler, pin) \
    pinMode(pin, INPUT_PULLUP); \
    attachInterrupt( \
        digitalPinToInterrupt(pin), \
        [](void){ \
            handler.toggle(); \
        }, \
        CHANGE \
    );

#define removePpmDecoderFromPin(handler, pin) \
    detachInterrupt(pin);

#ifdef PPM_PRINT_WARNINGS
#define SERIAL_CONNECTION
#endif

#endif