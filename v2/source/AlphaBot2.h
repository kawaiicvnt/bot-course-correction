#ifndef ALPHABOT2_H
#define ALPHABOT2_H

#include "CodalDevice.h"
#include <cstdint>

enum class Motors {
    M1 = 0x1,
    M2 = 0x2
};

enum class Sensor {
    Left = 0x1,
    Right = 0x2
};

enum class Dir {
    Forward = 0x1,
    Backward = 0x2,
    TurnRight = 0x3,
    TurnLeft = 0x4,
    Stop = 0x5
};

namespace codal {
    class AlphaBot2 : public CodalDevice {
    public:
        AlphaBot2();

        void MotorRun(Motors index, int speed);
        void Run(Dir index, int speed);
        void RunCourseCorrected(Dir index, int speed);
        void RunDelay(Dir index, int speed, int time);
        void RunCourseCorrectedDelay(Dir index, int speed, int time);
        
        bool Infrared(Sensor index);
        int Ultrasonic();
        
        int* AnalogRead();
        void SensorCalibrated();
        
        int* ReadSensorMax();
        int* ReadSensorMin();
        int* ReadCalibrated();
        int readLine();

    private:
        void initPCA9685();
        void setFreq(int freq);
        void setPwm(uint8_t channel, int on, int off);
        void i2cwrite(uint8_t addr, uint8_t reg, uint8_t value);
        uint8_t i2cread(uint8_t addr, uint8_t reg);

        int calibratedMax[5] = {650, 650, 650, 650, 650};
        int calibratedMin[5] = {100, 100, 100, 100, 100};
        int last_value = 0;
        bool initialized = false;

        static constexpr uint8_t PCA9685_ADDRESS = 0x40;
        static constexpr uint8_t MODE1 = 0x00;
        static constexpr uint8_t PRESCALE = 0xFE;
        static constexpr uint8_t LED0_ON_L = 0x06;
    };
}
#endif