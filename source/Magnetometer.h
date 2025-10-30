#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include "MicroBit.h"
using namespace std;
extern MicroBit uBit;

#define DEFAULT_SAMPLES 5

struct Vector3 {
    int x = 0;
    int y = 0;
    int z = 0;
    ManagedString toString() {
        return ("[" + ManagedString(x) + ", " + ManagedString(y) + ", " + ManagedString(z) + "]").toCharArray();
    }
    void operator += (Vector3 other){
        x += other.x;
        y += other.y;
        z += other.z;
    }
    void operator += (int i) {
        x += i;
        y += i;
        z += i;
    }
    void operator /= (Vector3 other) {
        x /= other.x;
        y /= other.y;
        z /= other.z;
    }
    void operator /= (int i) {
        x /= i;
        y /= i;
        z /= i;
    }
    Vector3 operator / (Vector3 other) {
        return Vector3{
        x / other.x,
        y / other.y,
        z / other.z };
    }
    Vector3 operator / (int i) {
        return Vector3{
            x / i,
            y / i,
            z / i };
    }
};

struct Rotation {
    float pitch = 0;
    float roll = 0;
    float yaw = 0;
    ManagedString toString() {
        return ("[" + ManagedString((int) pitch) + ", " + ManagedString((int) roll) + ", " + ManagedString((int) yaw) + "]").toCharArray();
    }
    Rotation toRads() {
        return Rotation{
            pitch * (float) (PI / 180),
            roll * (float) (PI / 180),
            yaw * (float) (PI / 180) };
    }
    Rotation toDegs() {
        return Rotation{
            pitch * (float) (180 / PI),
            roll * (float) (180 / PI),
            yaw * (float) (180 / PI) };
    }
    void operator += (Rotation other){
        pitch += other.pitch;
        roll += other.roll;
        yaw += other.yaw;
    }
    void operator += (int i) {
        pitch += i;
        roll += i;
        yaw += i;
    }
    void operator /= (Rotation other) {
        pitch /= other.pitch;
        roll /= other.roll;
        yaw /= other.yaw;
    }
    void operator /= (int i) {
        pitch /= i;
        roll /= i;
        yaw /= i;
    }
    Rotation operator / (Rotation other) {
        return Rotation{
        pitch / other.pitch,
        roll / other.roll,
        yaw / other.yaw };
    }
    Rotation operator / (int i) {
        return Rotation{
            pitch / i,
            roll / i,
            yaw / i };
    }
};

int calc_heading_vertical();
int calc_heading_vertical(Vector3 gauss_data);
int calc_heading_vertical_normalized(int samples);
int calc_heading_vertical_tilt();
int calc_heading_vertical_tilt(Vector3 gauss_data, Rotation rotational_data);
int calc_heading_vertical_tilt2(Vector3 gauss_data, Rotation rotational_data);
int calc_heading_vertical_tilt_normalized(int samples);
Rotation get_pitch_roll_vertical();
Vector3 get_gauss_data();
void compass_clock_fiber();
void clock_format(int heading);
Vector3 tilt_compensated_gauss_data(Vector3 gauss_data, Rotation rotational_data);
// Matrix precalculate_rotational_matrix(float pitch, float roll);
int tiltCompensatedBearing();

#endif