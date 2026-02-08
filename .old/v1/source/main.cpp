#include "MicroBit.h"
#include "Debug.h"
#include "Magnetometer.h"
#include "PinNames.h"
#include "main.h"

MicroBit uBit;
uint8_t speed = 100;
uint8_t tilt = 0;
uint8_t menu = 0;

ManagedString toHex(uint8_t var) {
    char hex[3];
    sprintf(hex, "%02x", var);
    return ManagedString(hex);
}

// Tilt is between 0 and 180 in an upright position
int calculateTiltAngle() {
    tilt = abs(atan2(uBit.accelerometer.getY(), -uBit.accelerometer.getX()) * (180/PI));
    return tilt;
}

static void on_command_receive(MicroBitEvent) {
    ManagedString data = uBit.serial.readUntil(ManagedString("\r\n"));
    printf(">> Command received: " + data + "\n");
    printf(ManagedString(uBit.serial.isReadable()) + "\n");
    if (!strcmp(data.toCharArray(), "H")) {
        printf("\nUsage:\n");
        printf("----------------------------------\n");
        printf("H - Print this help message\n");
        printf("C - Print compass heading\n");
        printf("R - Print rotational data\n");
        printf("G - Print gauss data\n");
        printf("CC - Course correct towards origin\n");
        printf("U - Print ultrasonic data\n");
        printf("I - Print infrared data\n");
        printf("RSM - Print sensor max\n");
        printf("RSN - Print sensor min\n");
        printf("TDS - Test data sending via serial\n\n");
    } else if (!strcmp(data.toCharArray(), "R")) {
        while (!(uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) == ManagedString("E"))) {
            Rotation data = get_pitch_roll_vertical();
            calculateTiltAngle();
            printf("Rotational: ");
            printf(data.toString());
            printf(" | Tilt: ");
            printf(tilt);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (!strcmp(data.toCharArray(), "G")) {
        while (!(uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) == ManagedString("E"))) {
            Vector3 gd = get_gauss_data();
            // Rotation rd_rads = get_pitch_roll_vertical().toRads();
            ManagedString ret = gd.toString();
            // ManagedString ret2 = tilt_compensated_gauss_data(gd, rd_rads).toString();
            printf("Gauss: ");
            printf(ret);
            // printf(" | Tilt Compensated: ");
            // printf(ret2);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (!strcmp(data.toCharArray(), "TDS")) {
        printf("Print int: ");
        printf(200);
        printf("\n");
        printf("Print int via ManagedString: " + ManagedString(200) + "\n");
        printf(("Print int via ManagedString.toCharArray(): " + ManagedString(200) + "\n").toCharArray());
        printf(uBit.compass.getX());
        printf("\n");
    }
}

static void forward(MicroBitEvent)
{
    calculateTiltAngle();
    printf(ManagedString(">> Sending forward command with speed: ") + speed + ManagedString(" and tilt: ") + tilt + ManagedString("\n"));
    uBit.radio.datagram.send("F" + toHex(speed) + toHex(tilt));
}
static void backward(MicroBitEvent)
{
    calculateTiltAngle();
    printf(ManagedString(">> Sending backward command with speed: ") + speed + ManagedString(" and tilt: ") + tilt + ManagedString("\n"));
    uBit.radio.datagram.send("B" + toHex(speed) + toHex(tilt));
}
static void stop(MicroBitEvent)
{
    // printf(">> Sending stop command\n");
    uBit.radio.datagram.send("S");
}

static void moveForward(MicroBitEvent)
{
    uBit.sleep(50);
    while (uBit.buttonB.isPressed() && !uBit.buttonA.isPressed()) {
        calculateTiltAngle();
        printf(ManagedString(">> Sending forward command with speed: ") + speed + ManagedString(" and tilt: ") + tilt + ManagedString("\n"));
        uBit.radio.datagram.send("F" + toHex(speed) + toHex(tilt));
        uBit.sleep(50);
    }
}

static void moveBackward(MicroBitEvent)
{
    uBit.sleep(50);
    while (uBit.buttonA.isPressed() && !uBit.buttonB.isPressed()) {
        calculateTiltAngle();
        printf(ManagedString(">> Sending backward command with speed: ") + speed + ManagedString(" and tilt: ") + tilt + ManagedString("\n"));
        uBit.radio.datagram.send("B" + toHex(speed) + toHex(tilt));
        uBit.sleep(50);
    }
}

static void changeSpeed(MicroBitEvent)
{
    speed = ((speed + 25) % 225) + 25;
    printf(ManagedString(">> Changing speed: ") + speed + ManagedString("\n"));
    uBit.display.printAsync((int) speed);
}

static void changeSpeed(bool increase) {
    if (increase) {
        speed += 25;
    } else {
        speed -= 25;
    }
    if (speed < 0) {
        speed = 0;
    } else if (speed > 250) {
        speed = 250;
    }
    printf(ManagedString(">> Changing speed: ") + speed + ManagedString("\n"));
    uBit.display.printAsync((int) speed);
}

static void increaseSpeed(MicroBitEvent)
{
    speed += 25;
    if (speed > 250) {
        speed = 250;
    }
    printf(ManagedString(">> Increasing speed: ") + speed + ManagedString("\n"));
    uBit.display.printAsync((int) speed);
}

static void decreaseSpeed(MicroBitEvent)
{
    speed -= 25;
    if (speed < 0) {
        speed = 0;
    }
    printf(ManagedString(">> Decreasing speed: ") + speed + ManagedString("\n"));
    uBit.display.printAsync((int) speed);
}

static void calibrate(MicroBitEvent)
{
    printf(">> Calibrating compass... ");
    uBit.compass.calibrate();
    printf("Done!\n");
}

static void controller() {
    while (1) {
        if (uBit.buttonAB.isPressed()) {
            uBit.display.image.setPixelValue(2, 2, 255);
            // Make sure we let go of AB first
            while (uBit.buttonAB.isPressed()) {
                uBit.sleep(50);
            }
            while (1) {
                if (uBit.buttonAB.isPressed()) {
                    break;
                } else if (uBit.buttonA.isPressed()) {
                    changeSpeed(false);
                } else if (uBit.buttonB.isPressed()) {
                    changeSpeed(true);
                }
                uBit.sleep(50);
            }
            uBit.display.image.clear();
        } else if (uBit.buttonA.isPressed()) {
            forward(MicroBitEvent());
        } else if (uBit.buttonB.isPressed()) {
            backward(MicroBitEvent());
        } else {
            stop(MicroBitEvent());
        }
        uBit.sleep(250);
    }
}

static void menuChange(MicroBitEvent)
{
    switch (menu) {
        case MENU_DEFAULT:
            uBit.display.image.clear();
            uBit.messageBus.ignore(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, decreaseSpeed);
            uBit.messageBus.ignore(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, increaseSpeed);
            uBit.messageBus.ignore(MICROBIT_ID_BUTTON_AB, MICROBIT_BUTTON_EVT_HOLD, calibrate);
            uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_DOWN, moveBackward);
            uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_DOWN, moveForward);
            uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_UP, stop);
            uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_UP, stop);
            menu++;
            break;
        case MENU_CHANGE_SPEED:
            uBit.display.printCharAsync('S');
            uBit.messageBus.ignore(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_DOWN, moveBackward);
            uBit.messageBus.ignore(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_DOWN, moveForward);
            uBit.messageBus.ignore(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_UP, stop);
            uBit.messageBus.ignore(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_UP, stop);
            uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_CLICK, decreaseSpeed);
            uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_CLICK, increaseSpeed);
            uBit.messageBus.listen(MICROBIT_ID_BUTTON_AB, MICROBIT_BUTTON_EVT_HOLD, calibrate);
            menu++;
            break;
        default:
            menu = 0;
    }
    if (menu > 1) {
        menu = 0;
    }
}

void debug_fiber() {
    uBit.serial.send("Debug fiber started!\n>> Type 'H' for help\n>> -----------------------------\n");
    // uBit.display.printAsync(uBit.serial.isReadable());
    // uBit.display.printAsync(uBit.serial.rxInUse());
    uBit.serial.clearRxBuffer();
    uBit.serial.setRxBufferSize(256);
    uBit.serial.eventOn(ManagedString("\r\n"));
    uBit.serial.isReadable();
    uBit.messageBus.listen(MICROBIT_ID_SERIAL, MICROBIT_SERIAL_EVT_DELIM_MATCH, on_command_receive);
    // uBit.display.printAsync(uBit.serial.isReadable());
}

int main()
{
    printf(">> Initializing microbit... ");
    uBit.init();
    uBit.serial.isReadable();
    printf("Done!\n");

    printf(">> Initializing radio... ");
    uBit.radio.enable();
    uBit.radio.setGroup(1);
    uBit.radio.setTransmitPower(7);
    uBit.radio.setFrequencyBand(0);
    printf("Done!\n");
    
    // printf(">> Setting up event listeners... ");
    // uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_DOWN, backward);
    // uBit.messageBus.listen(MICROBIT_ID_BUTTON_A, MICROBIT_BUTTON_EVT_UP, stop);
    // uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_DOWN, forward);
    // uBit.messageBus.listen(MICROBIT_ID_BUTTON_B, MICROBIT_BUTTON_EVT_UP, stop);
    // uBit.messageBus.listen(MICROBIT_ID_BUTTON_AB, MICROBIT_BUTTON_EVT_CLICK, changeSpeed);
    // uBit.messageBus.listen(MICROBIT_ID_BUTTON_AB, MICROBIT_BUTTON_EVT_LONG_CLICK, calibrate);
    // printf("Done!\n");
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_AB, MICROBIT_BUTTON_EVT_CLICK, menuChange);
    menuChange(MicroBitEvent());

    printf(">> Starting controller... ");
    // create_fiber(controller);
    printf("Done!\n");
    
    printf(">> Starting debug fiber... ");
    uBit.serial.isReadable();
    create_fiber(debug_fiber);

    
    // If main exits, there may still be other fibers running or registered event handlers etc.
    // Simply release this fiber, which will mean we enter the scheduler. Worse case, we then
    // sit in the idle task forever, in a power efficient sleep.
    release_fiber();
}

