#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include "Helper.h"
#include "ManagedString.h"

#define DEFAULT_SAMPLES 5
#define GAUSS_ALPHA 0.25 // Low-pass filter alpha value for gauss data

#define MAD_UPDATE_RATE 25 // Hz
#define MAD_UPDATE_PERIOD (1000 / MAD_UPDATE_RATE) // ms

using namespace codal;

// stores Magnetometer data in Rads
struct r_data {
    float pitch;
    float roll;
    float yaw;
    ManagedString toString(bool degrees = true) {
        if (degrees) {
            return ManagedString("[" + ftos(pitch * (180 / PI)) + ", " + ManagedString(ftos(roll * (180 / PI))) + ", " + ManagedString(ftos(yaw * (180 / PI))) + "]");
        } else {
            return ManagedString("[" + ftos(pitch) + ", " + ManagedString(ftos(roll)) + ", " + ManagedString(ftos(yaw)) + "]");
        }
    }
    
    // Simple constructor for Rotation
    r_data(float pitch, float roll, float yaw) : pitch(pitch), roll(roll), yaw(yaw) {}
    r_data() : pitch(0), roll(0), yaw(0) {}
};

struct g_data {
    float x;
    float y;
    float z;

    ManagedString toString() {
        return "[" + ftos(x) + ", " + ftos(y) + ", " + ftos(z) + "]";
    }

    g_data(float x, float y, float z) : x(x), y(y), z(z) {}
    g_data(int x, int y, int z) : x(x), y(y), z(z) {}
    g_data() : x(0), y(0), z(0) {}

    void lpf(g_data od, float alpha) {
        x = (x * alpha) + (od.x * (1 - alpha));
        y = (y * alpha) + (od.y * (1 - alpha));
        z = (z * alpha) + (od.z * (1 - alpha));
    }

    void normalize() {
        float length = sqrt(x * x + y * y + z * z);
        if (length != 0) {
            x = (x / length);
            y = (y / length);
            z = (z / length);
        }
    }

    g_data tilt_comp(r_data r) {
        // Tilt compensation using the rotation data
        g_data c;
        c.x = x * cos(r.pitch) + z * sin(r.pitch);
        c.y = y;
        c.z = -x * sin(r.pitch) + z * cos(r.pitch);
        return c;
    }

    r_data toRot() {
        return r_data(atan2(-z, y), atan2(x, y), 0);
    }

    // void operator = (g_data other) {
    //     x = other.x;
    //     y = other.y;
    //     z = other.z;
    // }
    void operator += (g_data other){
        x += other.x;
        y += other.y;
        z += other.z;
    }
    void operator += (int i) {
        x += i;
        y += i;
        z += i;
    }
    void operator /= (g_data other) {
        x /= other.x;
        y /= other.y;
        z /= other.z;
    }
    void operator /= (int i) {
        x /= i;
        y /= i;
        z /= i;
    }
    g_data operator / (g_data other) {
        return g_data{
        x / other.x,
        y / other.y,
        z / other.z };
    }
    g_data operator / (int i) {
        return g_data{
            x / i,
            y / i,
            z / i };
    }
};

class mag_acc_data {
public:
    g_data mag;
    g_data mag_t;
    g_data acc;
    r_data rot;
    // mag_acc_data();
    mag_acc_data();
    void init();
    void update();
    ManagedString toString();

    int heading();
    int heading_tilt();
}; // suddenly Italian ?

g_data get_mag_data(bool normalize = true);
g_data get_acc_data(bool normalize = true);
g_data get_mag_data(g_data old_data);
g_data get_acc_data(g_data old_data);
void update_gauss_data_fiber();

#endif
