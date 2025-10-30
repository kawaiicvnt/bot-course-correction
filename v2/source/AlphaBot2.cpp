#include "MicroBit.h"
#include "AlphaBot2.h"

extern MicroBit uBit;

AlphaBot2::AlphaBot2(){}

void AlphaBot2::i2cwrite(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[] = {reg, value};
    uBit.i2c.write(addr << 1, buf, 2);
}

uint8_t AlphaBot2::i2cread(uint8_t addr, uint8_t reg) {
    uBit.i2c.write(addr << 1, &reg, 1);
    uint8_t value;
    uBit.i2c.read(addr << 1, &value, 1);
    return value;
}

void AlphaBot2::initPCA9685() {
    i2cwrite(PCA9685_ADDRESS, MODE1, 0x00);
    setFreq(500);
    setPwm(0, 0, 4095);
    for (int i = 1; i < 16; i++) {
        setPwm(i, 0, 0);
    }
    initialized = true;
}

void AlphaBot2::setFreq(int freq) {
    float prescaleval = 25000000.0;
    prescaleval /= 4096.0;
    prescaleval /= freq;
    prescaleval -= 1.0;
    uint8_t prescale = (uint8_t)prescaleval;
    uint8_t oldmode = i2cread(PCA9685_ADDRESS, MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10;
    
    i2cwrite(PCA9685_ADDRESS, MODE1, newmode);
    i2cwrite(PCA9685_ADDRESS, PRESCALE, prescale);
    i2cwrite(PCA9685_ADDRESS, MODE1, oldmode);
    uBit.sleep(5);
    i2cwrite(PCA9685_ADDRESS, MODE1, oldmode | 0xA1);
}

void AlphaBot2::setPwm(uint8_t channel, int on, int off) {
    if (channel > 15) return; // No need to check below 0, as channel is unsigned
    uint8_t buf[] = {(uint8_t) (LED0_ON_L + 4U * channel), (uint8_t)(on & 0xFF), (uint8_t)((on >> 8) & 0xFF), (uint8_t)(off & 0xFF), (uint8_t)((off >> 8) & 0xFF)};
    uBit.i2c.write(PCA9685_ADDRESS << 1, buf, 5);
}

// For some reason the range in setPwm is a 13-bit value? WHY?
void AlphaBot2::MotorRun(Motors index, int speed) {
    if (!initialized) initPCA9685();

    speed *= 16; // TODO: probably remove this and go directly -4095, 4095
    speed = (speed > 4095) ? 4095 : ((speed < -4095) ? -4095 : speed);

    if (index == Motors::M1) {
        setPwm(2, 0, speed >= 0 ? 4095 : 0);
        setPwm(3, 0, speed >= 0 ? 0 : 4095);
        setPwm(1, 0, abs(speed));
    } else if (index == Motors::M2) {
        setPwm(5, 0, speed >= 0 ? 4095 : 0);
        setPwm(4, 0, speed >= 0 ? 0 : 4095);
        setPwm(6, 0, abs(speed));
    }
}

void AlphaBot2::Run(Dir index, int speed) {
    switch (index) {
        case Dir::Forward: MotorRun(Motors::M1, speed); MotorRun(Motors::M2, speed); break;
        case Dir::Backward: MotorRun(Motors::M1, -speed); MotorRun(Motors::M2, -speed); break;
        case Dir::TurnRight: MotorRun(Motors::M1, speed); MotorRun(Motors::M2, -speed); break;
        case Dir::TurnLeft: MotorRun(Motors::M1, -speed); MotorRun(Motors::M2, speed); break;
        case Dir::Stop: MotorRun(Motors::M1, 0); MotorRun(Motors::M2, 0); break;
    }
}

void AlphaBot2::RunCourseCorrected(Dir index, int speed) {
    // TODO: Implement or remove dumbass
}

void AlphaBot2::RunDelay(Dir index, int speed, int time) {
    Run(index, speed);
    uBit.sleep(time * 1000);
    Run(Dir::Stop, 0);
}

bool AlphaBot2::Infrared(Sensor index) {
    return (index == Sensor::Left) ? uBit.io.P12.getDigitalValue() == 0 : uBit.io.P16.getDigitalValue() == 0;
}

int AlphaBot2::Ultrasonic() {
    uBit.io.P1.setDigitalValue(0);
    uBit.sleep(2);
    uBit.io.P1.setDigitalValue(1);
    uBit.sleep(10);
    uBit.io.P1.setDigitalValue(0);
    return uBit.io.P2.getPulseUs(MICROBIT_PIN_EVT_RISE) / 58;
}

int* AlphaBot2::ReadSensorMax() { return calibratedMax; }
int* AlphaBot2::ReadSensorMin() { return calibratedMin; }

// Add AnalogRead(), SensorCalibrated(), ReadCalibrated(), and readLine() functions as needed

