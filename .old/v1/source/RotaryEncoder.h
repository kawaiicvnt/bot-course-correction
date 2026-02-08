#include "MicroBit.h"

#define ROTARY_ENCODER_UPDATE_INTERVAL 50 // ms

/*
 * Calculates, stores, and returns the position of a rotary encoder.
 * The position is relative to the last reset.
 * The position is in the range [0, 255].
 * The position is incremented when the rotary encoder is rotated clockwise.
 * The position is decremented when the rotary encoder is rotated counter-clockwise.
 * The position is reset to 0 when the reset() function is called.
 * The position is updated when the update() function is called.
 */
class RotaryEncoder {
private:
    MicroBitPin &pinA;      // Trigger pin. When this pin is pulled low, the rotary encoder is triggered.
    MicroBitPin &pinB;      // Rotation direction pin -> 0: clockwise, 1: counter-clockwise. The comparison values are based on pinA.
    MicroBitPin &pinC;      // Third pin (click)
    int position;           // [0,255] The position of the rotary encoder, relative to the last reset.
    bool lastState;         // The last state of the rotary encoder, used to determine the direction of rotation.
public:
    /*
     * Rotary encoder supports click functionality.
     */
    RotaryEncoder(MicroBitPin &a, MicroBitPin &b, MicroBitPin &c);
    /*
     * Rotary encoder does not support click functionality.
     */
    RotaryEncoder(MicroBitPin &a, MicroBitPin &b);
    void updateRotation(MicroBitEvent);
    int getPosition();
    void reset();
};

void fiber(RotaryEncoder* instance);