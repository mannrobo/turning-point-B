/**
 * motor.c - Improved motor control including slew rate, truespeed, deadband, and motor groups. Includes PID control for motors as well
 */

#pragma systemFile

#include "util.c"

// Stores motor targets, use this instead of motor[]
int motorTarget[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int motorSlew[10] = { 10, 127, 127, 127, 127, 127, 127, 127, 127, 10 };
int motorSlewLastSet[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int motorDeadband[10] = { 15, 15, 15, 15, 15, 15, 15, 15, 15, 15 };

// Logistic Curve to help with driving
int LOGISTIC[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 26, 27, 28, 29, 31, 32, 33, 34, 35, 37, 38, 39, 41, 42, 44, 45, 46, 48, 49, 51, 52, 54, 56, 57, 59, 60, 62, 64, 65, 67, 68, 70, 71, 73, 75, 76, 78, 79, 81, 82, 83, 85, 86, 88, 89, 90, 92, 93, 94, 95, 96, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 107, 108, 109, 110, 110, 111, 112, 113, 113, 114, 114, 115, 115, 116, 116, 117, 117, 118, 118, 119, 119, 119, 120, 120, 120, 121, 121, 121, 122, 122, 122, 122, 122, 123, 123, 123, 123, 123, 124, 124, 124, 124, 124, 124, 124, 125, 125, 125, 125, 127, 127, 127, 127, 127, 127, 127, 127, 127 };
int logistic(int n) {
  return sgn(n) * LOGISTIC[abs(n)];
}


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
