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

        robot.leftDrive = robot.driveController.output * 0.7;
        robot.rightDrive = robot.driveController.output * 0.7;
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

void driveCoast(int distance) {
    SensorValue[leftDrive] = 0;
    SensorValue[rightDrive] = 0;

    while(abs(SensorValue[leftDrive]) < abs(distance)) {
        robot.leftDrive = sgn(distance) * 90;
        robot.rightDrive = sgn(distance) * 90;

        wait1Msec(20);
    }

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

        // Turn at constant rate
        robot.leftDrive = -50 * sgn(degrees);  
        robot.rightDrive = 50 * sgn(degrees);
    } while(abs(robot.turnController.output) > 20);

    // Break
    robot.leftDrive = 50 * sgn(degrees);
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
    robot.firing = false;
}


// Fires two shots at two rpms
void doubleShot(int first, int second) {
    targetTBH(robot.flywheel, first);
    robot.intake = REVERSE;
    
    fire();
    
    targetTBH(robot.flywheel, second);

    wait1Msec(300);

    while(robot.ballLoaded) {
        robot.indexerOverride = FORWARD;
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
    targetTBH(robot.flywheel, 2400);

    wait1Msec(500);

    // Fire preload
    fire();

    wait1Msec(700);


    // Turn off flywheel to save power
    targetTBH(robot.flywheel, 0);
    wait1Msec(1000);

    // Drive to park
    driveMax(300);
    wait1Msec(400);


    if(match.alliance == ALLIANCE_RED) {
        turn(-100);
    } else {
        turn(100);
    }

    robot.leftDrive = -80;
    robot.rightDrive = -80;
    wait1Msec(200);
    robot.leftDrive = 0;
    robot.rightDrive = 0;

    wait1Msec(1000);

    driveMax(750)
}

void autonFrontfieldOld() {
    targetTBH(robot.flywheel, 2900);

    robot.intake = REVERSE;

    // Grab ball
    drive(1200);
    wait1Msec(400);
    drive(-1200);

    // Drive forward a bit
    drive(100);

    wait1Msec(1000);

    // Turn to face tree of flags
    if(match.alliance == ALLIANCE_RED) {
        turn(90);
    } else {
        turn(-90);
    };

    wait1Msec(500);

    doubleShot(2900, 0);

    wait1Msec(1000);

    // Turn to score caps
    if(match.alliance == ALLIANCE_RED) {
        turn(-45);
    } else {
        turn(45);
    };

    robot.intake = FORWARD;

    drive(400);

}

void autonFrontfield() {
    // Target flywheel early so we don't have to waste time waiit for it to spin up
    targetTBH(robot.flywheel, 2500);

    // Om nom nom balls (intake mode)
    robot.intake = REVERSE;

    // Grab low ball in front of you, scoring cap
    drive(550);
    drive(-650);

    wait1Msec(200);
    drive(100)
    // Turn to face tree of flags
    if(match.alliance == ALLIANCE_RED) {
        turn(100);
    } else {
        turn(-100);
    }

    // First first shot
    fire();

    // wait1Msec(300);
    // // set rpm for secound shot
    // // targetTBH(robot.flywheel, 2500);


    // // if(match.alliance == ALLIANCE_RED) {
    // //     turn(10);
    // // } else {
    // //     turn(-10);
    // // }

    // // // Score ground flag
    // drive(600);
    // wait1Msec(300);

    // // // Second shot
    // drive(-450)
    // drive(100)
    // fire();


}

// Fire Preload and Center Park
void autonProgSkills() {
    // Turn on flywheel
    targetTBH(robot.flywheel, 2700);
    wait1Msec(3000);

    // Fire preload
    fire();

    wait1Msec(3000);


    // Turn off flywheel to save power
    targetTBH(robot.flywheel, 0);
    wait1Msec(1000);

    // Drive to park
    driveMax(400);
    wait1Msec(300);

    turn(-90);

    robot.leftDrive = -80;
    robot.rightDrive = -80;
    wait1Msec(200);
    robot.leftDrive = 0;
    robot.rightDrive = 0;

    wait1Msec(1000);

    driveMax(2000)

}

void autonBlake() {
    // Target flywheel early so we don't have to waste time waiting for it to spin up
    targetTBH(robot.flywheel, 2800);

    robot.intake = REVERSE;

    // Om nom nom balls (intake mode)
    drive(900);
    turn(71);


    wait1Msec(1000)


    fire();

    targetTBH(robot.flywheel, 2600);
    wait1Msec(4000)
    fire();
    

}


void autonTestFlywheel() {
    targetTBH(robot.flywheel, 2500);
    robot.intake = FORWARD;
    while(true) {
        fire();
    }
}

void autonDoubleShot()  {
    doubleShot(2500, 2000)
}

void autonTestDrive() {
    targetTBH(robot.flywheel, 2500)
    drive(600);
}