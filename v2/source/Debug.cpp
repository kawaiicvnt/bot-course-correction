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
    // printf("receiving message... ");
    ManagedString data = uBit.serial.readUntil(ManagedString("\r\n"));
    // printf("received!\n");
    // printf(ManagedString("Command: ") + data + "\n");
    if (data == ManagedString("C")) {
        mag_acc_data gd1 = mag_acc_data(), gd2 = mag_acc_data(); // Separate instance for debugging
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            gd1.update();
            gd2.update();
            int D1 = mad.heading();
            int D2 = gd2.heading_tilt();
            printf("H/NT: ");
            printf(D1);
            printf(" | H/WT: ");
            printf(D2);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("R")) {
        g_data accel_data = get_acc_data(true);
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            accel_data = get_acc_data(accel_data);
            ManagedString ret = accel_data.toRot().toString();
            printf("Rotational: ");
            printf(ret);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("M")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            ManagedString ret = get_mag_data(false).toString();
            printf("Magnetometer: ");
            printf(ret);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("A")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            ManagedString ret = get_acc_data(false).toString();
            printf("Accelerometer: ");
            printf(ret);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("MN")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            g_data data = get_mag_data(true);
            ManagedString ret = data.toString();
            printf("Magnetometer: ");
            printf(ret);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("AN")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            g_data data = get_acc_data(true);
            ManagedString ret = data.toString();
            printf("Accelerometer: ");
            printf(ret);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("MNL")) {
        g_data data = get_mag_data(true);
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            data = get_mag_data(data);
            ManagedString ret = data.toString();
            printf("Magnetometer: ");
            printf(ret);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("ANL")) {
        g_data data = get_acc_data(true);
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            data = get_acc_data(data);

            ManagedString ret = data.toString();
            printf("Accelerometer: ");
            printf(ret);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("GC")) {
        printf("Calibration data: \n");
        CompassCalibration calibration = uBit.compass.getCalibration();
        Sample3D centre = calibration.centre;
        Sample3D scale = calibration.scale;
        printf("Centre: ");
        printf(ManagedString(centre.x) + ", " + ManagedString(centre.y) + ", " + ManagedString(centre.z) + "\n");
        printf("Scale: " + ManagedString(scale.x) + ", " + ManagedString(scale.y) + ", " + ManagedString(scale.z) + "\n");
        printf("Radius: " + ManagedString(calibration.radius) + "\n");
    
    } else if (data == ManagedString("CC")) {
        Robot robot;
        robot.set_speed(100, 100, 0);
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            robot.course_correct_towards_origin();
            printf(robot.toString());
            printf("\n");
            uBit.sleep(200);
        }
    } else if (data.charAt(0) == 'C' && data.charAt(1) == 'D') {
        packeddata pdata[DEBUG_DATA_MAX_SAMPLES];
        int i = 0;
        printf("Ouu husbanto, yu risena tu mucha femtanyal, we aru homuresu\n");
        bool is_acc = data.charAt(2) == 'A';
        bool is_mag = data.charAt(2) == 'M';
        bool is_lpf = data.charAt(3) == 'L';
        
        g_data gauss = get_acc_data(true);
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            if (i >= DEBUG_DATA_MAX_SAMPLES) {
                uBit.sleep(100);
                continue; // Prevent overflow
            }
            fraction_to_display(i, DEBUG_DATA_MAX_SAMPLES);
            if (is_acc) {
                if (is_lpf)
                    gauss = get_acc_data(gauss);
                else
                    gauss = get_acc_data(false);
                pdata[i] = packeddata(0, 0, gauss.x, gauss.y, gauss.z);
            } else if (is_mag) {
                if (is_lpf)
                    gauss = get_mag_data(gauss);
                else
                    gauss = get_mag_data(false);
                pdata[i] = packeddata(0, 0, gauss.x, gauss.y, gauss.z);
            }
            gauss = get_acc_data(true);
            i++;
            if (i >= DEBUG_DATA_MAX_SAMPLES) { // Only scroll once
                uBit.display.scroll("Overflow!");
            }
            uBit.sleep(DEBUG_DATA_SAMPLE_PERIOD);
        }
        if (is_acc) {
            printf("Accelerometer");
        } else if (is_mag) {
            printf("Magnetometer");
        }
        printf(" data collected:" + ManagedString(i) + "samples\n");
        printf("sl;sr;x;y;z\n");
        for (int j = 0; j < i; j++) {
            printf(pdata[j].toString());
        }
    } else if (data == ManagedString("U")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            int usonic = alphabot.Ultrasonic();
            printf("Ultrasonic: ");
            printf(usonic);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("I")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            bool left = alphabot.Infrared(Sensor::Left);
            bool right = alphabot.Infrared(Sensor::Right);
            printf("Left: ");
            printf(left);
            printf(" | Right");
            printf(right);
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("RSMX")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            int *data = alphabot.ReadSensorMax();
            printf("Sensor Max: ");
            for (int i = 0; i < 5; i++) {
                printf(data[i]);
                printf(" ");
            }
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("RSMN")) {
        while (uBit.serial.readUntil(ManagedString("\r\n"), ASYNC) != ManagedString("E")) {
            int *data = alphabot.ReadSensorMin();
            printf("Sensor Min: ");
            for (int i = 0; i < 5; i++) {
                printf(data[i]);
                printf(" ");
            }
            printf("\n");
            uBit.sleep(100);
        }
    } else if (data == ManagedString("TDS")) {
        printf("Print int: ");
        printf(200);
        printf("\n");
        printf("Print int via ManagedString: " + ManagedString(200) + "\n");
        printf(("Print int via ManagedString.toCharArray(): " + ManagedString(200) + "\n").toCharArray());
        printf(uBit.compass.getX());
        printf("\n");
    } else {
        uBit.serial.send("\n");
        if (data != ManagedString("H") && data != ManagedString("HELP")
         && data != ManagedString("h") && data != ManagedString("help")
         && data != ManagedString("?"))
            uBit.serial.send("\nUnknown command: " + data + "\n");
        printf("Available commands:\n");
        printf("----------------------------------\n");
        printf("H - Print this help message\n");
        printf("C - Print compass heading\n");
        printf("R - Print rotational data (normalized & lpf)\n");
        printf("M - Print magnetometer data\n");
        printf("MN - Print normalized magnetometer data\n");
        printf("MNL - Print normalized + lpf magnetometer data\n");
        printf("A - Print accelerometer data\n");
        printf("AN - Print normalized accelerometer data\n");
        printf("ANL - Print normalized + lpf accelerometer data\n");
        printf("GC - Print compass calibration data\n");
        printf("CC - Course correct towards origin\n");
        printf("U - Print ultrasonic data\n");
        printf("I - Print infrared data\n");
        printf("RSMX - Print sensor max\n");
        printf("RSMN - Print sensor min\n");
        printf("TDS - Test data sending via serial\n\n");
    }
}

void setup_debugger() {
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