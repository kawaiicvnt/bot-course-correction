#ifndef DEBUG_H
#define DEBUG_H

#include <cstdint>
#include "MicroBitCompat.h"

using namespace codal;

#define printf(...) uBit.serial.send(__VA_ARGS__)

#define DEBUG_DATA_SAMPLE_RATE 100 // Hz
#define DEBUG_DATA_SAMPLE_PERIOD (1000 / DEBUG_DATA_SAMPLE_RATE) // ms
#define DEBUG_DATA_MAX_SAMPLES 15 * DEBUG_DATA_SAMPLE_RATE // 60 seconds worth of samples

void on_command_receive(MicroBitEvent);
void setup_debugger();
void compass_clock_fiber();
void clock_format(int heading);

struct packeddata {
    // SL=[-128,127], SR=[-128,127], X=[0,255], Y=[0,255], Z=[0,255]
    // LLLLLLLLRRRRRRRRRRXXXXXXXXYYYYYYYYZZZZZZZZ
    // L - 8bit R - 8bit X - 8bit Y - 8bit Z - 8bit
    int8_t sl;
    int8_t sr;
    uint8_t x;
    uint8_t y;
    uint8_t z;
    packeddata() : sl(0), sr(0), x(0), y(0), z(0) {}
    // Packs the data down to one byte per variable and everything is an int lol
    packeddata(int sl, int sr, float x, float y, float z) {
        this->sl = sl/2;
        this->sr = sr/2;
        this->x = x/4;
        this->y = y/4;
        this->z = z/4;
    }
    ManagedString toString() {
        return ManagedString(ManagedString(sl) + ";" + ManagedString(sr) + ";" + ManagedString(x) + ";" + ManagedString(y) + ";" + ManagedString(z) + "\n");
    }
};
#endif
