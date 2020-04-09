#include "ppm.h"


static volatile unsigned long* timer;
static volatile int* data_ptr;
static volatile int* chan_ptr;
static volatile int* sync_ptr;
static int* pin_ptr;

static void high();
static void low();

ppmDecoder::ppmDecoder(int pin) {
    timer = new unsigned long;
    data_ptr = new int[14];
    chan_ptr = new int;
    sync_ptr = new int;
    pin_ptr = new int;
    
    *pin_ptr = pin;

    is_synced = false;
    is_connected = false;
}

void ppmDecoder::begin() {

    pinMode(*pin_ptr, INPUT_PULLUP);

    delay(250);

    sync_ptr = data_ptr;
    chan_ptr = data_ptr;

    attachInterrupt(digitalPinToInterrupt(*pin_ptr), [](void){
        *chan_ptr = micros() - *timer;
        *timer = micros();
        if (chan_ptr == data_ptr + 13) chan_ptr = data_ptr;
        else ++chan_ptr;
    }, CHANGE);

    sync();
}

void ppmDecoder::sync() {
    for (int i = 0; i < MAXIMUM_RESYNC_ATTEMPTS; ++i) {

        if (*sync_ptr > MIN_SYNC_PULSEWIDTH) {
            is_synced = true;
            break;
        }

        if (sync_ptr == data_ptr + 13) sync_ptr = data_ptr;
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
    if (sync_ptr + chan <= data_ptr + 13) {
        return *(sync_ptr + chan);
    }
    else {
        return *(sync_ptr + chan - 14);
    }
}

bool ppmDecoder::error() {
    return !is_connected || !is_synced;
}

ppmDecoder::~ppmDecoder() {
    delete timer;
    delete data_ptr;
    delete sync_ptr;
    delete chan_ptr;
    delete pin_ptr;
}

static void high() {
    *timer = micros();
    attachInterrupt(digitalPinToInterrupt(*pin_ptr), low, LOW);
}

static void low() {
    *chan_ptr = micros() - *timer;
    attachInterrupt(digitalPinToInterrupt(*pin_ptr), high, HIGH);
    if (chan_ptr == data_ptr + 6) chan_ptr = data_ptr;
    else ++chan_ptr;
}