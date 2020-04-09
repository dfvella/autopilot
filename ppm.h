#ifndef PPM_H
#define PPM_H

#include <Arduino.h>

#define MIN_SYNC_PULSEWIDTH 10000

#define MAXIMUM_RESYNC_ATTEMPTS 13

#define PPM_PRINT_WARNINGS

#define TELEMETRY_CHECKING

class ppmDecoder {
    public:
        ppmDecoder(int pin);
        void begin();
        void sync();
        bool error();
        int get(int chan);
        ~ppmDecoder();

        static constexpr int ARL = 2;
        static constexpr int ELE = 4;
        static constexpr int THR = 6;
        static constexpr int RUD = 8;
        static constexpr int GER = 10;
        static constexpr int AUX = 12;

    private:
        bool is_synced;
        bool is_connected;
};

#endif