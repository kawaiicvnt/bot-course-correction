#include "Magnetometer.h"
#include "ManagedString.h"
#include "MicroBit.h"

extern MicroBit uBit; // The micro:bit object
extern mag_acc_data mad;

mag_acc_data::mag_acc_data() {}

void mag_acc_data::init() {
    mag = get_mag_data(true);
    mag.normalize(); // 
    acc = get_acc_data(true);
    acc.normalize();
    rot = acc.toRot();
    mag_t = mag.tilt_comp(rot);
}

void mag_acc_data::update() {
    mag = get_mag_data(mag);
    acc = get_acc_data(acc);
    rot = acc.toRot();
    mag_t = mag.tilt_comp(rot);
};

ManagedString mag_acc_data::toString() {
    return ManagedString("M: " + mag.toString() + "\nA: " + acc.toString() + "\nR: " + rot.toString(true) + "\n");
}

//  We calculate the arctan of two axes' values, to figure the heading
//  For horizontal orientation, this would be Y, X.
//  For vertical, we swap the Y and Z axes. So -Z X.
//  I suspect it's not possible to calculate with a tilt setup,
//  as the accelerometer works well only for horizontal orientation AFAIK.
//  Additionally, the magnetometer appears to be very sensitive.
//  If playing with LSBs doesn't help, I could capture 3-5 frames and average them. Maybe even weighted avg?
//  Another concern, would be that since the microbit will be on the AlphaBot2, the ride will be.. bumpy
//  Which means that data will be messed up. Probably weighted normalization and data outlier rejection can help.
//  Also once the AlphaBot2 goes on a sloped surface, all bets are off. Tilt-compensation is REQUIRED.
//  I'll need to project a 3D vector on 2D plane, before using the 2D formula. I'll need the accelerometer for that.
int mag_acc_data::heading() {
    int D = (int) (atan2(mag.y, -mag.z) * (180 / PI));
    if (D > 360) {
        D = D - 360;
    } else if (D < 0) {
        D = D + 360;
    }
    return D;
}

int mag_acc_data::heading_tilt() {
    // Calculate the tilt-compensated magnetometer vector (we only care about x and z for our use-case)
    float x = mag.x * cos(rot.roll) - mag.y * sin(rot.roll);
    float z = mag.z * cos(rot.pitch) + mag.y * sin(rot.pitch);

    int deg = (int) (atan2(x, z) * (180 / PI));
    if (deg > 360) {
        deg = deg - 360;
    } else if (deg < 0) {
        deg = deg + 360;
    }
    return deg;
}

// Non class stuff that helps feed and process data

// Normalizes by default
g_data get_mag_data(bool normalize) {
    g_data data = g_data(uBit.compass.getX(), uBit.compass.getY(), uBit.compass.getZ());
    if (normalize) {
        data.normalize();
    }
    return data;
}

// Normalizes and applies low-pass filter
g_data get_mag_data(g_data old_data) {
    g_data data = get_mag_data(true);
    data.lpf(old_data, GAUSS_ALPHA);
    return data;
}

// Normalizes by default
g_data get_acc_data(bool normalize) {
    g_data vector = {uBit.accelerometer.getX(), uBit.accelerometer.getY(), uBit.accelerometer.getZ()};
    if (normalize) {
        vector.normalize();
    }
    return vector;
}

// Normalizes and applies low-pass filter
g_data get_acc_data(g_data old_data) {
    g_data data = get_acc_data(true);
    data.lpf(old_data, GAUSS_ALPHA);
    return data;
}

void update_gauss_data_fiber() {
    mad.init(); // Initialize the class holding our data hostage
    while(true) {
        mad.update();
        uBit.sleep(MAD_UPDATE_PERIOD);
    }
}