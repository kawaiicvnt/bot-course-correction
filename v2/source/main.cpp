// external deps
#include "CodalFiber.h"
#include "Compass.h"
#include "Magnetometer.h"
#include "ManagedString.h"
#include "MicroBit.h"
#include "MicroBitDevice.h"
#include "MicroBitEvent.h"
#include "MicroBitUARTService.h"
#include <cstdint>
#include <stdio.h>
// internal deps
#include "AlphaBot2.h"
#include "Course.h"
#include "Debug.h"

#define DEBUGLINEWIDTH 60

// Assuming AlphaBot2 constructor takes no arguments or different arguments
AlphaBot2 alphabot;
mag_acc_data mad; // M.A.D. - Magnetometer and Accelerometer Data
MicroBitUARTService *ble_uart; // The BLE UART service. We can send and receive serial data to and from the connected device, by using this service.
MicroBit uBit; // The microbit board definition

// Default calibration data
CompassCalibration* def_calib = new CompassCalibration(
    Sample3D(-30716, 36320, -60056),
    Sample3D(1096, 1103, 1093),
    65903
    );

int speed = 0;
float steer_weight = 1;

struct robot_speed {
  int16_t speed_l;
  int16_t speed_r;
};

int fromHex(ManagedString str) {
    int value = 0;
    for (int i = 0; i < str.length(); i++) {
        char c = str.charAt(i);
        if (c >= '0' && c <= '9') {
            value = value * 16 + (c - '0');
        } else if (c >= 'A' && c <= 'F') {
            value = value * 16 + (c - 'A' + 10);
        } else if (c >= 'a' && c <= 'f') {
            value = value * 16 + (c - 'a' + 10);
        }
    }
    return value;
}

struct Speed {
    int speed_l;
    int speed_r;
};

// Determines headroom in each motor and takes advantage of it to steer
Speed determine_motor_speed_still(int speed, int tilt) {
    int speed_l, speed_r;

    if (85 < tilt && tilt < 95) { // Ignore small tilt
        speed_l = speed;
        speed_r = speed;
    } else if (tilt < 90) { // Left -> Cut power to left motor, increase power to right motor
        speed_l = ((-255 - speed) * (90 - tilt) / speed) + speed;
        speed_r = ((255 - speed) * (90 - tilt) / speed) + speed;
    } else { // Right -> Cut power to right motor, increase power to left motor
        speed_l = ((255 - speed) * (90 - tilt) / speed) + speed;
        speed_r = ((-255 - speed) * (90 - tilt) / speed) + speed;
    }

    return Speed{speed_l, speed_r};
}

Speed determine_motor_speed_moving(int tar_speed, int tilt, bool forward) {
    float speed_l, speed_r;

    if (85 < tilt && tilt < 95) { // Ignore small tilt
        speed_l = tar_speed;
        speed_r = tar_speed;
    } else if (tilt < 90) { // Left -> Cut power to left motor, increase power to right motor
        speed_l = (-(((float) (tar_speed)) * ((90.0f - (float) tilt)/90.0f) * steer_weight) + tar_speed);
        speed_r = ((((float) (255 - tar_speed)) * ((90.0f - (float) tilt)/90.0f) * steer_weight) + tar_speed);
    } else { // Right -> Cut power to right motor, increase power to left motor
        speed_l = ((((float) (255 - tar_speed)) * (((float) tilt - 90.0f)/90.0f) * steer_weight) + tar_speed);
        speed_r = (-(((float) (tar_speed)) * (((float) tilt - 90.0f)/90.0f) * steer_weight) + tar_speed);
    }
    Speed sp = Speed{(int) speed_l, (int) speed_r};

    return sp;
}

void move(MicroBitEvent) {
    ManagedString data = ble_uart->readUntil(ManagedString("\r\n"), SYNC_SLEEP);
    printf(ManagedString("> BLE received: ") + data);
    if (data == "LS") {
        alphabot.MotorRun(Motors::M1, 0);
    } else if (data == "RS") {
        alphabot.MotorRun(Motors::M2, 0);
    } else if (data == "LG") {
        alphabot.MotorRun(Motors::M1, speed);
    } else if (data == "RG") {
        alphabot.MotorRun(Motors::M2, speed);
    } else if (data.charAt(0) == 'S') {
        alphabot.MotorRun(Motors::M1, 0);
        alphabot.MotorRun(Motors::M2, 0);
    } else if (data.charAt(0) == 'F') {
        Speed speed = determine_motor_speed_moving(fromHex(data.substring(1, 2)), fromHex(data.substring(3, 2)), true);
        printf(ManagedString("--> ") + data.substring(0,1) + ", " + data.substring(1, 2) + ", " + data.substring(3, 2) + " ");
        printf(ManagedString("--> F | Speed -> L: ") + speed.speed_l + ManagedString(" | R: ") + speed.speed_r);
        alphabot.MotorRun(Motors::M1, speed.speed_l);
        alphabot.MotorRun(Motors::M2, speed.speed_r);
    } else if (data.charAt(0) == 'B') {
        Speed speed = determine_motor_speed_moving(fromHex(data.substring(1, 2)), fromHex(data.substring(3, 2)), false);
        printf(ManagedString("--> ") + data.substring(0,1) + ", " + data.substring(1, 2) + ", " + data.substring(3, 2) + " ");
        printf(ManagedString("--> B | Speed -> L: ") + speed.speed_l + ManagedString(" | R: ") + speed.speed_r);
        alphabot.MotorRun(Motors::M1, -speed.speed_r);
        alphabot.MotorRun(Motors::M2, -speed.speed_l);
    }
    uBit.display.printCharAsync('O', 100);
}

void changeSpeed(MicroBitEvent) {
    speed = (speed + 50) % 255;
    uBit.display.printAsync(speed);
}

int main()
{
    uBit.init();
    printf(ManagedString("\n") + ( DEBUGLINEWIDTH * ManagedString("-") ) + "\n"); // Line separator
    printf("> Initialized uBit!\n");

    // This is data we've extracted via debugging tools from a running
    printf("> Setting up pre-calculated calibration data... ");
    uBit.compass.setCalibration(*def_calib);
    printf("Done!\n");

    // Set the rotation of the device, to upright, facing away from the user.
    // To ensure that we calculate the correct compass heading.
    printf("> Setting up magnetometer and accelerometer fiber... ");
    mad.init();
    create_fiber(update_gauss_data_fiber);
    printf("Done!\n");

    printf("> Setting up debugger event... ");
    setup_debugger();
    printf("Done!\n");

    printf("> Setting up BLE UART service... ");
    uBit.bleManager.init("botto(m)", uBit.getSerial(), uBit.messageBus, uBit.storage, false);
    uBit.bleManager.setTransmitPower(7);
    uBit.bleManager.advertise();
    ble_uart = new MicroBitUARTService(*uBit.ble, 32, 32);
    ble_uart->eventOn("\r\n");
    printf("Done!\n");


    printf("> Setting up event listeners... ");
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, [](MicroBitEvent) {
        Robot robot;
        // We wanna stay still and let the AB2 correct it's heading.
        // It should apply equal, but opposite speed to each wheel
        robot.set_speed(0, 0, 0);
        uBit.sleep(200);
        int iterations = 0;
        while (!uBit.buttonB.isPressed()) {
            robot.course_correct_towards_origin();
            alphabot.MotorRun(Motors::M1, robot.cur_speed_l);
            alphabot.MotorRun(Motors::M2, robot.cur_speed_r);
            // We update course correction every 10ms, but don't want to clutter our serial.
            if (iterations++ > 10) {
                printf(robot.toString() + ManagedString("\n"));
                iterations = 0;
            }
            uBit.sleep(25);
        }
        // Turn off the motors lol
        alphabot.MotorRun(Motors::M1, 0);
        alphabot.MotorRun(Motors::M2, 0);
        uBit.sleep(100);
    });
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, [](MicroBitEvent) {
        alphabot.MotorRun(Motors::M1, 0);
        alphabot.MotorRun(Motors::M2, 0);
    });
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_AB, MICROBIT_BUTTON_EVT_CLICK, [](MicroBitEvent) {
        uBit.compass.calibrate();
    });
    uBit.messageBus.listen(MICROBIT_ID_BLE_UART, MICROBIT_UART_S_EVT_DELIM_MATCH, move);
    printf("Done!\n");

    printf("> Finished setup!\n");
    printf(( DEBUGLINEWIDTH * ManagedString("-") ) + "\n"); // Line separator
    release_fiber();
}
