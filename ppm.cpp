#include "ppm.h"


ppmDecoder::ppmDecoder(int pin_in) { 
    pin = pin_in;

    is_synced = false;
    is_connected = false;

    sync_ptr = data;
    chan_ptr = data;

    pinMode(pin, INPUT_PULLUP);
}

void ppmDecoder::sync() {
    for (int i = 0; i < MAXIMUM_RESYNC_ATTEMPTS; ++i) {

        if (*sync_ptr > MIN_SYNC_PULSEWIDTH) {
            is_synced = true;
            break;
        }

        if (sync_ptr == data + 13) sync_ptr = data;
        else ++sync_ptr;

        is_synced = false;

        #ifdef PPM_PRINT_WARNINGS
        Serial.println("ppm out of sync");
        #endif 
    }
    #ifdef TELEMETRY_CHECKING
    static int count = 0;

    static int prev = *sync_ptr;

    if (*sync_ptr - prev < 20) ++count;
    else count = 0;

    prev = *sync_ptr;

    if (count > 30) is_connected = false;
    else is_connected = true;
    #endif
}

int ppmDecoder::get(int chan) {
    if (sync_ptr + chan <= data + 13) {
        return *(sync_ptr + chan);
    }
    else {
        return *(sync_ptr + chan - 14);
    }
}

bool ppmDecoder::error() {
    return !is_connected || !is_synced;
}

void ppmDecoder::toggle() {
    *chan_ptr = micros() - timer;
    timer = micros();
    if (chan_ptr == data + 13) chan_ptr = data;
    else ++chan_ptr;
}