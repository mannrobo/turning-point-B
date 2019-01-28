/**
 * motor.c - Improved motor control including slew rate, truespeed, deadband, and motor groups. Includes PID control for motors as well
 */

#include "util.c"

// Stores motor targets, use this instead of motor[]
int motorTarget[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int motorSlew[10] = { 10, 10, 127, 127, 10, 127, 127, 127, 127, 10 };
int motorSlewLastSet[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int motorDeadband[10] = { 15, 15, 15, 15, 15, 15, 15, 15, 15, 15 };

// Thanks to Jess from 21S (@Unionjackz) for truespeed tables
static const char L298[128] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 11, 11, 12,
    12, 12, 12, 13, 13, 13, 14, 14, 14, 14,
    15, 15, 15, 16, 16, 16, 17, 17, 17, 18,
    18, 18, 19, 19, 19, 20, 20, 20, 21, 21,
    22, 22, 23, 23, 23, 24, 25, 25, 26, 26,
    26, 27, 27, 28, 28, 29, 29, 30, 30, 31,
    31, 32, 32, 33, 33, 34, 35, 35, 35, 35,
    35, 35, 38, 38, 39, 41, 41, 41, 42, 43,
    45, 46, 47, 47, 48, 49, 49, 50, 51, 52,
    53, 54, 55, 56, 58, 59, 63, 66, 67, 70,
    73, 74, 75, 78, 80, 84, 87, 88, 92, 95,
    97, 100, 105, 108, 114, 117, 121, 122, 124, 127
};

static const char MC29[128] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 13, 13, 14,
    14, 14, 14, 15, 15, 15, 15, 16, 16, 16,
    16, 16, 17, 17, 17, 17, 17, 17, 17, 17,
    18, 18, 18, 18, 18, 18, 18, 19, 19, 19,
    19, 19, 20, 20, 20, 20, 21, 21, 21, 22,
    22, 22, 22, 23, 23, 23, 23, 24, 24, 24,
    25, 25, 25, 26, 26, 26, 27, 27, 27, 28,
    28, 28, 29, 29, 30, 30, 30, 31, 31, 32,
    32, 33, 33, 34, 34, 35, 36, 36, 37, 37,
    38, 39, 40, 40, 41, 42, 43, 44, 45, 46,
    46, 48, 49, 50, 51, 53, 55, 56, 58, 60,
    62, 64, 67, 70, 72, 76, 79, 83, 84, 127,
};

void motorControlStep() {
    int outs[10]; // Stores intermediate output values
    int motorCurrent; // Temp variable used for slew rate
    // Loop through each motor slot
    for(int i = 0; i < 10; i++) {

        // 1. Target
        outs[i] = clamp(motorTarget[i], -127, 127);

        // 2. Deadband
        if (motorDeadband[i] > abs(outs[i])) {
            outs[i] = 0;
        }

        // 3. Slew Rate - Gradually increases motor power, reducing the chance of PTC trips
        motorCurrent = motorSlewLastSet[i];
        if(motorCurrent != outs[i]) {
            motorCurrent +=
            sgn(outs[i] - motorCurrent) * // Whether to increase or decrease in value
            clamp(motorSlew[i], 0, abs(outs[i] - motorCurrent)); // The amount to increase, the clamp prevents the value from being greater than the difference remaining
        }

        outs[i] = motorCurrent;
        motorSlewLastSet[i] = outs[i];

        // 4. TrueSpeed - Standardizes the acceleration curve of the Motor Controller
        // if(i == 0 || i == 9) {
        //     outs[i] = sgn(outs[i]) * L298[abs(outs[i])];
        // } else {
        //     outs[i] = sgn(outs[i]) * MC29[abs(outs[i])];
        // }

        // 5. Set Motor
        motor[i] = outs[i];
    }
}
