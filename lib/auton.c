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

#pragma systemFile

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
    int auton; // Auton Selected (see menu)
} matchConfiguration;

matchConfiguration match;


// Drives a specific distance (forward, use negative for backwards) in ticks
void drive(int distance) {

    SensorValue[leftDrive] = 0;
    SensorValue[rightDrive] = 0;

    // Configure PID
    configurePID(robot.driveController, 1.6, 0, 0);
    targetPID(robot.driveController, distance);

    do {
        robot.driveController.value = SensorValue[leftDrive];
        stepPID(robot.driveController);

        robot.leftDrive = robot.driveController.output;
        robot.rightDrive = robot.driveController.output;
    } while(abs(robot.driveController.output) > 20);

    // Break
    robot.leftDrive  = -50 * sgn(distance);
    robot.rightDrive = -50 * sgn(distance);
    wait1Msec(200);
    robot.leftDrive = 0;
    robot.rightDrive = 0;

}

void driveMax(int distance) {

    SensorValue[leftDrive] = 0;
    SensorValue[rightDrive] = 0;

    while(abs(SensorValue[leftDrive]) < abs(distance)) {
        robot.leftDrive = sgn(distance) * 127;
        robot.rightDrive = sgn(distance) * 127;

        wait1Msec(20);
    }

    // Break
    robot.leftDrive  = -50 * sgn(distance);
    robot.rightDrive = -50 * sgn(distance);
    wait1Msec(200);
    robot.leftDrive = 0;
    robot.rightDrive = 0;

}


/**
 * Find the "absolute" gyro position (Always 0 - 360) in DEGREES!
 */
float absoluteDirection(int measurement) {
    return ((measurement < 0 ? 3600 + measurement : measurement) % 3600) / 10.0;
}

/**
 * Turns to face a particular degree mark (0 = forward, 90 = left?)
 * NOTE: As per reccomendation by 5225A, turns are ABSOLUTE.
 * This means a call to turn(0) will translate to the robot
 * turning to face its starting position
 * Negative degrees will turn left, and positive degrees will turn right
 */
void turn(int degrees) {

    // Configure PID
    configurePID(robot.turnController, 1, 0, 0);
    targetPID(robot.turnController, degrees);

    SensorValue[gyro] = 0;

    do {
        robot.turnController.value = SensorValue[gyro] / 10.0;
        stepPID(robot.turnController);

        robot.leftDrive = -robot.turnController.output;
        robot.rightDrive = robot.turnController.output;
    } while(abs(robot.turnController.output) > 20);

    robot.leftDrive  = 50 * sgn(degrees);
    robot.rightDrive = -50 * sgn(degrees);
    wait1Msec(100);
    robot.leftDrive = 0;
    robot.rightDrive = 0;
}

void fire() {
    robot.firing = true;
    while(robot.firing)  {
        wait1Msec(20);
    }
}

void wallSquare(int direction) {
    robot.leftDrive = 80 * direction;
    robot.rightDrive = 80 * direction;
    wait1Msec(500);
    robot.leftDrive = 0;
    robot.rightDrive = 0;
}


/**
 * Routines!
 */


// Backfield Auton
void autonBackfield() {
    // Turn on flywheel
    targetTBH(robot.flywheel, 2000);

    wait1Msec(500);

    // Fire preload
    fire();

    wait1Msec(700);


    // Turn off flywheel to save power
    targetTBH(robot.flywheel, 0);
    wait1Msec(1000);

    // Drive to park
    driveMax(700);
    wait1Msec(400);


    if(match.alliance == ALLIANCE_RED) {
        turn(-90);
    } else {
        turn(90);
    }

    robot.leftDrive = -80;
    robot.rightDrive = -80;
    wait1Msec(200);
    robot.leftDrive = 0;
    robot.rightDrive = 0;

    wait1Msec(1000);

    driveMax(1400)
}

void autonFrontfield() {
    targetTBH(robot.flywheel, 2500);

    robot.intake = REVERSE;

    // Score low flag
    drive(400);
    wait1Msec(400);
    drive(-100);

    // Shoot at middle flag
    fire();

    wait1Msec(500);
    drive(-100);

    // Score closest cap
    if(match.alliance == ALLIANCE_RED) {
        turn(-90);
    } else {
        turn(90);
    }

    drive(400);
}

// Fire Preload and Center Park
void autonProgSkills() {
    // Turn on flywheel
    targetTBH(robot.flywheel, 2000);

    wait1Msec(2000);

    // Fire preload
    fire();


    // Turn off flywheel to save power
    targetTBH(robot.flywheel, 0);
    wait1Msec(1000);

    // Drive to park
    driveMax(500);
    wait1Msec(400);

    turn(-90);

    robot.leftDrive = -80;
    robot.rightDrive = -80;
    wait1Msec(200);
    robot.leftDrive = 0;
    robot.rightDrive = 0;

    wait1Msec(1000);

    driveMax(2000)

}



void autonTestFlywheel() {
    targetTBH(robot.flywheel, 2500);
    robot.intake = FORWARD;
    while(true) {
        fire();
    }
}

void autonDoubleShot()  {
    targetTBH(robot.flywheel);
    robot.intake = FORWARD;

    fire();

    // Cut power to flywheel
    targetTBH(robot.flywheel, 0);

    // Fire second shot
    robot.indexerOverride = FORWARD
}