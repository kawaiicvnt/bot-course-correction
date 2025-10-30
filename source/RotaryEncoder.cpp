#include "MicroBit.h"
#include "RotaryEncoder.h"

RotaryEncoder::RotaryEncoder(MicroBitPin &a, MicroBitPin &b, MicroBitPin &c): pinA(a), pinB(b), pinC(c), position(0), lastState(0) {
    lastState = pinA.getDigitalValue();
    uBit.messageBus.listen(pinA.getDigitalValue(), MICROBIT_PIN_EVT_FALL, this, &RotaryEncoder::updateRotation);
}

RotaryEncoder::RotaryEncoder(MicroBitPin &a, MicroBitPin &b): pinA(a), pinB(b), pinC(*(new MicroBitPin(-1, PinName(NC), PIN_CAPABILITY_STANDARD))), position(0), lastState(0) {
    lastState = pinA.getDigitalValue();

}

// Is called on a falling edge of pinA.
void RotaryEncoder::updateRotation(MicroBitEvent) {
    bool currentState = pinA.getDigitalValue() == 1 ? true : false;
    bool bState = pinB.getDigitalValue() == 1 ? true : false;
    if (bState != currentState && position <= 255) { // Clockwise
        position++;
    } else if (bState == currentState && position >= 0) { // Counter-clockwise
        position--;
    }
    lastState = currentState;
}

int RotaryEncoder::getPosition() {
    return position;
}

void RotaryEncoder::reset() {
    position = 0;
}

extern MicroBit uBit;

// void fiber(void* instance) {
//     RotaryEncoder* encoder = (RotaryEncoder*) instance;
//     while (1) {
//         encoder->updateRotation();
//         uBit.sleep(ROTARY_ENCODER_UPDATE_INTERVAL);
//     }    
// }