#include "Magnetometer.h"
 

// //  We calculate the arctan of two axes' values, to figure the heading
// //  For horizontal orientation, this would be Y, X.
// //  For vertical, we swap the Y and Z axes. So -Z X.
// //  I suspect it's not possible to calculate with a tilt setup,
// //  as the accelerometer works well only for horizontal orientation AFAIK.
// //  Additionally, the magnetometer appears to be very sensitive.
// //  If playing with LSBs doesn't help, I could capture 3-5 frames and average them. Maybe even weighted avg?
// //  Another concern, would be that since the microbit will be on the AlphaBot2, the ride will be.. bumpy
// //  Which means that data will be messed up. Probably weighted normalization and data outlier rejection can help.
// //  Also once the AlphaBot2 goes on a sloped surface, all bets are off. Tilt-compensation is REQUIRED.
// //  I'll need to project a 3D vector on 2D plane, before using the 2D formula. I'll need the accelerometer for that.
int calc_heading_vertical() {
    int xGaussData = uBit.compass.getX();
    int yGaussData = uBit.compass.getY();
    int zGaussData = uBit.compass.getZ();
    int D = (int) (atan2(xGaussData, -zGaussData) * (180 / PI));
    if (D > 360) {
        D = D - 360;
    } else if (D < 0) {
        D = D + 360;
    }
    return D;
}

int calc_heading_vertical(Vector3 gauss_data) {
    int D = (int) (atan2(gauss_data.x, -gauss_data.z) * (180 / PI));
    if (D > 360) {
        D = D - 360;
    } else if (D < 0) {
        D = D + 360;
    }
    return D;
}

int calc_heading_vertical_normalized(int samples) {
    Vector3 data = {0, 0, 0};
    for (int i = 0; i < samples; i++) {
        data += get_gauss_data();
    }
    data /= samples;
    int heading = calc_heading_vertical(data);
    return heading;
}

int calc_heading_vertical_tilt() {
    Vector3 gauss_data = get_gauss_data();
    Rotation rotational_data = get_pitch_roll_vertical();
    Vector3 tilt_compensated_data = tilt_compensated_gauss_data(gauss_data, rotational_data);
    int D = (int) (atan2(tilt_compensated_data.x, -tilt_compensated_data.z) * (180 / PI));
    if (D > 360) {
        D = D - 360;
    } else if (D < 0) {
        D = D + 360;
    }
    return D;
}

int calc_heading_vertical_tilt(Vector3 gauss_data, Rotation rotational_data) {
    Vector3 tilt_compensated_data = tilt_compensated_gauss_data(gauss_data, rotational_data);
    int D = (int) (atan2(tilt_compensated_data.x, -tilt_compensated_data.z) * (180 / PI));
    if (D > 360) {
        D = D - 360;
    } else if (D < 0) {
        D = D + 360;
    }
    return D;
}

int calc_heading_vertical_tilt_normalized(int samples) {
    Vector3 gauss = {0, 0, 0};
    Rotation rotation = {0, 0, 0};
    for (int i = 0; i < samples; i++) {
        gauss += get_gauss_data();
        rotation += get_pitch_roll_vertical();
    }
    gauss /= samples;
    rotation /= samples;
}

Vector3 get_gauss_data() {
    Vector3 data = {uBit.compass.getX(), uBit.compass.getY(), uBit.compass.getZ()};
    return data;
}

void compass_clock_fiber() {
    while (true) {
        // TODO: Add rounding
        int heading = calc_heading_vertical_normalized(DEFAULT_SAMPLES) / 22.5;
        // heading %= 16; // clock format
        clock_format(heading);
        uBit.sleep(100);
    }
}

void clock_format(int clk) {
    uBit.display.clear();
    uBit.display.image.setPixelValue(2,2,255);
    switch (clk) {
        case 0:
            uBit.display.image.setPixelValue(2,0,255);
            break;
        case 1:
            uBit.display.image.setPixelValue(3,0,255);
            break;
        case 2:
            uBit.display.image.setPixelValue(4,0,255);
            break;
        case 3:
            uBit.display.image.setPixelValue(4,1,255);
            break;
        case 4:
            uBit.display.image.setPixelValue(4,2,255);
            break;
        case 5:
            uBit.display.image.setPixelValue(4,3,255);
            break;
        case 6:
            uBit.display.image.setPixelValue(4,4,255);
            break;
        case 7:
            uBit.display.image.setPixelValue(3,4,255);
            break;
        case 8:
            uBit.display.image.setPixelValue(2,4,255);
            break;
        case 9:
            uBit.display.image.setPixelValue(1,4,255);
            break;
        case 10:
            uBit.display.image.setPixelValue(0,4,255);
            break;
        case 11:
            uBit.display.image.setPixelValue(0,3,255);
            break;
        case 12:
            uBit.display.image.setPixelValue(0,2,255);
            break;
        case 13:
            uBit.display.image.setPixelValue(0,1,255);
            break;
        case 14:
            uBit.display.image.setPixelValue(0,0,255);
            break;
        case 15:
            uBit.display.image.setPixelValue(1,0,255);
            break;
    }
}

Rotation get_pitch_roll_vertical() {
    int pitch = atan2(-uBit.accelerometer.getZ(), uBit.accelerometer.getY()) * (180/PI);
    // int pitch = atan2(uBit.accelerometer.getX(), sqrt(uBit.accelerometer.getY() * uBit.accelerometer.getY() + uBit.accelerometer.getZ() * uBit.accelerometer.getZ())) * (180/PI);
    int roll = atan2(uBit.accelerometer.getX(), uBit.accelerometer.getY()) * (180/PI);
    // In a vertical position, yaw is irrelevant since we don't have any gravitational pull on the Z axis
    Rotation data = {pitch, roll, 0};
    return data;
}

Vector3 get_accel_values() {
    Vector3 vector = {uBit.accelerometer.getX(), uBit.accelerometer.getY(), uBit.accelerometer.getZ()};
    return vector;
}

// Vector3 tilt_compensated_gauss_data(Vector3 gauss_data, Rotation rotational_data) {
//     Matrix rot = precalculate_rotational_matrix(rotational_data.pitch, rotational_data.roll);
//     Matrix data = Matrix(gauss_data);
//     Vector3 gauss_data_tilt = (rot * data).toVector3();

//     return gauss_data_tilt;
// }

// // Based on https://www.pololu.com/file/0J434/LSM303DLH-compass-app-note.pdf
// Matrix precalculate_rotational_matrix(float pitch, float roll) {
//     Matrix rot = Matrix(3, 3);

//     // We precalculate the sin and cos of the pitch and roll angles
//     // Once, to avoid extra cycles, especially since they're floats :)
//     float cos_pitch = cos(pitch);
//     float sin_pitch = sin(pitch);
//     float cos_roll = cos(roll);
//     float sin_roll = sin(roll);
    
//     // First row
//     rot.a[0] = cos_pitch;
//     rot.a[1] = 0;
//     rot.a[2] = sin_pitch;
//     // Second row
//     rot.a[3] = sin_roll * sin_pitch;
//     rot.a[4] = cos_roll;
//     rot.a[5] = -sin_roll * cos_pitch;
//     // Third row
//     rot.a[6] = -cos_roll * sin_pitch;
//     rot.a[7] = sin_roll;
//     rot.a[8] = cos_roll * cos_pitch;

//     return rot;
// }

int calc_heading_vertical_tilt2(Vector3 gauss, Rotation rot) {
    float cos_pitch = cos(rot.pitch);
    float sin_pitch = sin(rot.pitch);
    float cos_roll = cos(rot.roll);
    float sin_roll = sin(rot.roll);

    // Tilt compensation
    // float x = gauss.x * cos_pitch + gauss.y * sin_pitch * sin_roll + gauss.z * sin_pitch * cos_roll;
    // float y = gauss.y * cos_roll - gauss.z * sin_roll;
    // float z = -gauss.x * sin_pitch + gauss.y * cos_pitch * sin_roll + gauss.z * cos_pitch * cos_roll;
    float x = gauss.x * cos_pitch - gauss.z * sin_pitch * sin_roll - gauss.y * sin_pitch * cos_roll;
    float y = -gauss.z * cos_roll + gauss.y * sin_roll;
    float z = -gauss.x * sin_pitch - gauss.z * cos_pitch * sin_roll - gauss.y * cos_pitch * cos_roll;

    int D = (int) (atan2(x, y) * (180 / PI));
    if (D > 360) {
        D = D - 360;
    } else if (D < 0) {
        D = D + 360;
    }
    return D;
}

int tiltCompensatedBearing()
{
    // Precompute the tilt compensation parameters to improve readability.
    float phi = uBit.accelerometer.getRollRadians();
    float theta = uBit.accelerometer.getPitchRadians();

    Vector3 s = get_gauss_data();

    float x = (float) s.x;
    float y = (float) s.y;
    float z = (float) s.z;

    // Precompute cos and sin of pitch and roll angles to make the calculation a little more efficient.
    float sinPhi = sinf(phi);
    float cosPhi = cosf(phi);
    float sinTheta = sinf(theta);
    float cosTheta = cosf(theta);

     // Calculate the tilt compensated bearing, and convert to degrees.
    float bearing = (360.0f*atan2f(x*cosTheta + y*sinTheta*sinPhi + z*sinTheta*cosPhi, z*sinPhi - y*cosPhi)) / (2.0f*(float)PI);

    // Handle the 90 degree offset caused by the NORTH_EAST_DOWN based calculation.
    bearing = 90.0f - bearing;

    // Ensure the calculated bearing is in the 0..359 degree range.
    if (bearing < 0)
        bearing += 360.0f;

    return (int) (bearing);
}