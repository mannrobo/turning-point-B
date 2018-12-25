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


/**
 * Find the "absolute" gyro position (Always 0 - 360)
 */
int gyroAbsolute(tSensors gyro) {
    return ((SensorValue[gyro] < 0 ? 3600 - SensorValue[gyro] : SensorValue[gyro]) % 3600) / 10;
}

/**
 * Turns to face a particular degree mark (0 = forward, 90 = left?)
 * NOTE: As per reccomendation by 5225A, turns are ABSOLUTE.
 * This means a call to turn(0) will translate to the robot
 * turning to face its starting position
 * Negative degrees will turn left, and positive degrees will turn right
 */
void turn(int degrees) {
    // Convert target and actual to absolute degree indications (0-360 degrees)
    int target = (degrees < 0 ? 360 - degrees : degrees) % 360;
    int inital  = gyroAbsolute(gyro);

    // Decide the direction to turn based on start and target (target > start means positive multiplier)
    int direction = sgn(target - inital);


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

    if(match.alliance == ALLIANCE_BLUE) {
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

void autonSquareDance() {
    turn(90);
    drive(300);
}
