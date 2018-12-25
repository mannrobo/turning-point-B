/**
 * Utilies for autonomous programs
 */

#include "../hal.c"
#include "pid.c"
#include "lcd.c"

#define ALLIANCE_RED  0
#define ALLIANCE_BLUE 1


#define TURBO 2.4
#define HIGHSPEED 1.6
#define TORQUE 1

/**
 * Converts inches to quadrature encoder ticks, for the purpose of setting targets in PIDs
 * Usage:
 *  inchesToTicks(10, 3.25, 1, TURBO);
 *
 * @param float unit The number inches to convert to ticks
 * @param float wheelDiameter The diameter of the wheel, in inches
 * @param float gearing The external gear ration (Speed Up => x, Torque Up => 1/x)
 * @param float motorGear The internal gear ratio of the motor (constants TURBO, HIGHSPEED, and TORQUE have been specified)
 */
float inchesToTicks(float unit, float wheelDiameter, float gearing, float motorGear) {
    return (
        (unit / (motorGear * gearing)) / // Cancel out gear ratio
        (wheelDiameter * PI) // Divide by circumfrence to solve for rotations
    ) * 360; // 360 ticks in a rotation
}

typedef struct {
    int alliance; // RED is 0, BLUE is 1
} matchConfiguration;

matchConfiguration match;


// Drives a specific distance (forward, use negative for backwards) in ticks
void drive(int distance) {

    SensorValue[leftDrive] = 0;
    SensorValue[rightDrive] = 0;

    while(abs(SensorValue[leftDrive]) < abs(distance)) {
        robot.leftDrive = sgn(distance) * 80;
        robot.rightDrive = sgn(distance) * 80;

        wait1Msec(20);
    }

    robot.leftDrive = 0;
    robot.rightDrive = 0;

}


// Turns using gyro
void turn(float degrees) {
    configurePID(robot.turnController, 1.9, 0, 0);
    targetPID(robot.turnController, degrees);

    do {
        robot.turnController.value = SensorValue[gyro] / 10;
        stepPID(robot.turnController);

        // Clamp turns even if the error is greater. This will work to prevent overshoot
        robot.leftDrive = -clamp(abs(robot.turnController.output), 0, 50) * sgn(robot.turnController.output);
        robot.rightDrive = clamp(abs(robot.turnController.output), 0, 50) * sgn(robot.turnController.output);

    } while(abs(robot.turnController.error) > 50)

    robot.leftDrive = 0;
    robot.rightDrive = 0;
}

void fire() {
    robot.firing = true;
    while(robot.ballLoaded)  {
        wait1Msec(20);
    }
}


/**
 * Routines!
 */

// Routine 1: Closest to flag

void autonOne() {
    // Turn on Flywheel and start intake
    targetTBH(robot.flywheel, 3200);
    robot.intake = FORWARD;

    writeDebugStreamLine("forward");
    drive(1100);
    writeDebugStreamLine("back");
    drive(-1100);

    writeDebugStreamLine("turn");

    if(match.alliance = ALLIANCE_BLUE) {
        turn(-90);
    } else {
        turn(90);
    }

    writeDebugStreamLine("fire");

    fire();
}


void autonTestFlywheel() {
    targetTBH(robot.flywheel, 2500);
    robot.intake = FORWARD;
    while(true) {
        fire();
    }
}