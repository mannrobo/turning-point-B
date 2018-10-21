/**
 * Hardware Abstraction Layer
 */

#include "lib\motor.c"

typedef struct robotState {
    int leftDrive;
    int rightDrive;
} robotState;

robotState robot;

/** Section 1: Drive **/
void driveHandle() {
    motorTarget[DriveFrontLeft] = robot.leftDrive;
    motorTarget[DriveBackLeft] = robot.leftDrive;

    motorTarget[DriveFrontRight] = robot.rightDrive;
    motorTarget[DriveBackRight] = robot.rightDrive;
}

/** Section 2: Controller **/
void controllerHandle() {

    // Drive
    int forward = abs(vexRT[Ch3]) > 60 ? vexRT[Ch3] : 0,
        turn = abs(vexRT[Ch4]) > 90 ? vexRT[Ch4] * 0.9 : 0,
        left = forward + turn,
        right = forward - turn;

    robot.leftDrive = sgn(left) * rescaleTo(127, abs(left), abs(right), 0);
    robot.rightDrive = sgn(right) * rescaleTo(127, abs(left), abs(right), 1);

}


task handleAll() {
    while(true)  {
        controllerHandle();
        driveHandle();
        motorHandle();
        wait1Msec(20);
    }
}
