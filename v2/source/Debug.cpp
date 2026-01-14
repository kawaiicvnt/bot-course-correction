#include "AlphaBot2.h"
#include "Course.h"
#include "Debug.h"
#include "Magnetometer.h"
#include "ManagedString.h"
#include "MicroBit.h"
#include <cstdint>

extern AlphaBot2 alphabot;
extern MicroBit uBit;
extern mag_acc_data mad;

// Displays a fraction 
static void fraction_to_display(int value, int max_value) {
    uBit.display.clear();
    if (max_value == 0) {
        uBit.display.clear();
        uBit.display.scroll("ERR MAX=0");
        return;
    }
    value = (value * 25) / max_value;

    uBit.display.clear();
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            if (value >= (i + 1))
                uBit.display.image.setPixelValue(i, j, 255);
            else
                uBit.display.image.setPixelValue(i, j, 0);
        }
    }
}

void on_command_receive(MicroBitEvent) {
    // PRINT("receiving message... ");
    ManagedString data = uBit.serial.readUntil(ManagedString("\r\n"));
    ManagedString ret;
    // PRINT("received!\n");
    // PRINT(ManagedString("Command: ") + data + "\n");
    if (data == ManagedString("C")) {
        mag_acc_data gd1 = mag_acc_data(), gd2 = mag_acc_data(); // Separate instance for debugging
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            gd1.update();
            gd2.update();
            int D1 = mad.heading();
            int D2 = gd2.heading_tilt();
            PRINT("H/NT: ");
            PRINT(D1);
            PRINT(" | H/WT: ");
            PRINT(D2);
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("R")) {
        g_data accel_data = get_acc_data(true);
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            accel_data = get_acc_data(accel_data);
            ret = accel_data.toRot().toString();
            PRINT("Rot: ");
            PRINT(ret);
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("M")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            ret = get_mag_data(false).toString();
            PRINT("Mag: ");
            PRINT(ret);
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("A")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            ret = get_acc_data(false).toString();
            PRINT("Acc: ");
            PRINT(ret);
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("MN")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            g_data data = get_mag_data(true);
            ret = data.toString();
            PRINT("Mag: ");
            PRINT(ret);
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("AN")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            g_data data = get_acc_data(true);
            ret = data.toString();
            PRINT("Acc: ");
            PRINT(ret);
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("MNL")) {
        g_data data = get_mag_data(true);
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            data = get_mag_data(data);
            ret = data.toString();
            PRINT("Mag: ");
            PRINT(ret);
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("ANL")) {
        g_data data = get_acc_data(true);
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            data = get_acc_data(data);
            ret = data.toString();
            PRINT("Acc: ");
            PRINT(ret);
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("GC")) {
        PRINT("Calib data: \n");
        CompassCalibration calibration = uBit.compass.getCalibration();
        Sample3D centre = calibration.centre;
        Sample3D scale = calibration.scale;
        PRINT("Centre: ");
        PRINT(ManagedString(centre.x) + ", " + ManagedString(centre.y) + ", " + ManagedString(centre.z) + "\n");
        PRINT("Scale: " + ManagedString(scale.x) + ", " + ManagedString(scale.y) + ", " + ManagedString(scale.z) + "\n");
        PRINT("Radius: " + ManagedString(calibration.radius) + "\n");
    
    } else if (data == ManagedString("CC")) {
        Robot robot;
        robot.set_speed(100, 100, 0);
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            robot.course_correct_towards_origin();
            PRINT(robot.toString());
            PRINT("\n");
            uBit.sleep(200);
        }
    } else if (data == ManagedString("U")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            int usonic = alphabot.Ultrasonic();
            PRINT("Ultrasonic: ");
            PRINT(usonic);
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("I")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            bool left = alphabot.Infrared(Sensor::Left);
            bool right = alphabot.Infrared(Sensor::Right);
            PRINT("Left: ");
            PRINT(left);
            PRINT(" | Right");
            PRINT(right);
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("RSMX")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            int *data = alphabot.ReadSensorMax();
            PRINT("Sensor Max: ");
            for (int i = 0; i < 5; i++) {
                PRINT(data[i]);
                PRINT(" ");
            }
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("RSMN")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            int *data = alphabot.ReadSensorMin();
            PRINT("Sensor Min: ");
            for (int i = 0; i < 5; i++) {
                PRINT(data[i]);
                PRINT(" ");
            }
            PRINT("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("TDS")) {
        PRINT("Print int: ");
        PRINT(200);
        PRINT("\n");
        PRINT("Print int via ManagedString: " + ManagedString(200) + "\n");
        PRINT(("Print int via ManagedString.toCharArray(): " + ManagedString(200) + "\n").toCharArray());
        PRINT(uBit.compass.getX());
        PRINT("\n");
    } else {
        uBit.serial.send("\n");
        if (data != ManagedString("H") && data != ManagedString("HELP")
         && data != ManagedString("h") && data != ManagedString("help")
         && data != ManagedString("?"))
            uBit.serial.send("\nUnknown command: " + data + "\n");
        PRINT("Available commands:\n");
        PRINT("----------------------------------\n");
        PRINT("H - Print this help message\n");
        PRINT("C - Print compass heading\n");
        PRINT("R - Print rotational data (normalized & lpf)\n");
        PRINT("M - Print magnetometer data\n");
        PRINT("MN - Print normalized magnetometer data\n");
        PRINT("MNL - Print normalized + lpf magnetometer data\n");
        PRINT("A - Print accelerometer data\n");
        PRINT("AN - Print normalized accelerometer data\n");
        PRINT("ANL - Print normalized + lpf accelerometer data\n");
        PRINT("GC - Print compass calibration data\n");
        PRINT("CC - Course correct towards origin\n");
        PRINT("U - Print ultrasonic data\n");
        PRINT("I - Print infrared data\n");
        PRINT("RSMX - Print sensor max\n");
        PRINT("RSMN - Print sensor min\n");
        PRINT("TDS - Test data sending via serial\n\n");
    }
}

void setupDebugger() {
    // uBit.serial.send("> Debug fiber started!\n");
    uBit.serial.init();
    uBit.serial.isReadable();
    uBit.serial.eventOn(ManagedString("\r\n"));
    uBit.serial.init();
    uBit.messageBus.listen(MICROBIT_ID_SERIAL, MICROBIT_SERIAL_EVT_DELIM_MATCH, on_command_receive);
}

void compass_clock_fiber() {;
    while (true) {
        // TODO: Add rounding
        // Magnetometer                          // Accelerometer
        int heading = mad.heading_tilt() / 22.5;
        // int heading = calc_heading_vertical_normalized(DEFAULT_SAMPLES) / 22.5;
        // heading %= 16; // clock format
        clock_format(heading);
        uBit.sleep(100);
    }
}

void clock_format(int clk) {
    uBit.display.clear();
    uBit.display.image.setPixelValue(2,2,255); // Center pixel for clock face
    // Map the clock value to a specific pixel on the display
    const uint8_t pixelmap[16][2] = {  {2, 0}, {3, 0}, {4, 0}, // Top row
                    {4, 1}, {4, 2}, {4, 3}, {4, 4}, // Right column
                    {3, 4}, {2, 4}, {1, 4}, {0, 4}, // Bottom row
                    {0, 3}, {0, 2}, {0, 1}, {0, 0}, // Left column
                    {1, 0} };                       // Connect back to top row
    uBit.display.image.setPixelValue(pixelmap[clk][0], pixelmap[clk][1], 255);
}