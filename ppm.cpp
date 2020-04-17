#include "ppm.h"


ppmDecoder::ppmDecoder() :
    is_synced(true), 
    is_connected(true),
    sync_ptr(data), 
    chan_ptr(data) { }

void ppmDecoder::sync() {
    for (int i = 0; i < MAXIMUM_RESYNC_ATTEMPTS; ++i) {

        if (*sync_ptr > MIN_SYNC_PULSEWIDTH) {
            is_synced = true;
            break;
        }

        if (sync_ptr == data + 13) sync_ptr = data;
        else ++sync_ptr;

        is_synced = false;
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

    #ifdef PPM_PRINT_WARNINGS
    if (!is_synced) Serial.println("PPM: out of sync");
    #ifdef TELEMETRY_CHECKING
    if (!is_connected) Serial.println("PPM: telemetry lost");
    #endif
    #endif
}

int ppmDecoder::get(int chan) {
    int val;
    if (sync_ptr + chan <= data + 13) {
        val = *(sync_ptr + chan);
    }
    else {
        val = *(sync_ptr + chan - 14);
    }
    val += 300;
    if (val < MINIMUM_PPM_PUSLEWIDTH) {
        val = MINIMUM_PPM_PUSLEWIDTH;
    }
    if (val > MAXIMUM_PPM_PULSEWIDTH) {
        val = MAXIMUM_PPM_PULSEWIDTH;
    }
    return val;
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