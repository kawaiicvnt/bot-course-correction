#ifndef COURSE_H
#define COURSE_H

#define AB2_SPEED_MAX 255
#define AB2_SPEED_MIN -255
#define AB2_LR_THRESHOLD 10 // TODO: If I go for a 13-bit value, I should set thresh to 160
#define AB2_HEADING_THRESHOLD 5

#include "Magnetometer.h"

extern mag_acc_data ma_d;

/*
 * Represents the robot's current state.
 * 
 * @param cur_sleed_l The current speed of the left motor.
 * @param cur_speed_r The current speed of the right motor.
 * @param org_speed_l The original speed of the left motor.
 * @param org_speed_r The original speed of the right motor.
 * @param org_heading The original heading of the robot.
 * @param weight How hard the robot will steer.
 */
struct Robot {
    int cur_speed_l = 0;
    int cur_speed_r = 0;
    int org_speed_l = 0;
    int org_speed_r = 0;
    int org_heading = 0;
    int weight = 1; // How hard it will steer

    /*
    * Sets the speed of the robot. Saves the speed and custom heading as original.
    * If L and R speed difference is within a certain threshold.
    * Used when starting to move in a straight line.
    * 
    * @param robot The robot to set the speed of.
    * @param speed_l The speed of the left motor.
    * @param speed_r The speed of the right motor.
    * @param heading The custom heading of the robot.
    */
    void set_speed(int speed_l, int speed_r, int heading) {
        int speed_diff = abs(speed_l - speed_r);
        if (speed_diff > AB2_LR_THRESHOLD) {
            int speed = (speed_l + speed_r) / 2;
            cur_speed_l = speed;
            org_speed_l = speed;
            cur_speed_r = speed;
            org_speed_r = speed;
        } else {
            cur_speed_l = speed_l;
            org_speed_l = speed_l;
            cur_speed_r = speed_r;
            org_speed_r = speed_r;
        }
        org_heading = heading;
    }

    /*
    * Sets the speed of the robot. Saves the speed and current heading as original.
    * If L and R speed difference is within a certain threshold.
    * Used when starting to move in a straight line.
    * 
    * @param robot The robot to set the speed of.
    * @param speed_l The speed of the left motor.
    * @param speed_r The speed of the right motor.
    */    
    void set_speed(int speed_l, int speed_r) {
        int heading = ma_d.heading_tilt();
        // int heading = calc_heading_vertical_normalized(DEFAULT_SAMPLES);
        set_speed(speed_l, speed_r, heading);
    }

    /*
    * Corrects the course of the robot.
    * 
    * @param robot The pointer of the robot to correct.
    * @param heading_offset The offset to correct the heading by.
    */
    void course_correct(int heading_offset) {
        // bool dir = heading_offset > 0;
        float power = ((float) abs(heading_offset)) / 180;
        if (abs(heading_offset) < AB2_HEADING_THRESHOLD) { // Make sure it's worth correcting
            cur_speed_l = org_speed_l;
            cur_speed_r = org_speed_r;
        } else if (heading_offset > 0) { // Turn right
            cur_speed_l = power * ((255 - org_speed_l)) + org_speed_l;
            cur_speed_r = power * ((-255 - org_speed_r)) + org_speed_r;
        } else { // Turn left
            cur_speed_l = power * ((-255 - org_speed_l)) + org_speed_l;
            cur_speed_r = power * ((255 - org_speed_r)) + org_speed_r;
        }
    }

    int calc_heading_offset(int origin, int heading) {
        int heading_offset = heading - origin;
        if (heading_offset > 180) {
            heading_offset = heading_offset - 360;
        } else if (heading_offset < -179) {
            heading_offset = heading_offset + 360;
        }
        return heading_offset;
    }

    // TODO: Since this does not have a loop and cannot handle data, 
    void course_correct_towards_origin() {
        int heading = ma_d.heading_tilt();
        int offset = calc_heading_offset(org_heading, heading);
        course_correct(offset);
    }

    ManagedString toString() {
        return ("Robot: { L: " + ManagedString(cur_speed_l) + ", R: " + ManagedString(cur_speed_r) + ", Offset: " + ManagedString(calc_heading_offset(org_heading, ma_d.heading_tilt())) + " }");
    }

};

#endif
