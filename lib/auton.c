/**
 * Utilies for autonomous programs
 */

#include "../hal.c"
#include "pid.c"


PIDController driveController;


// Drives a specific distance (forward, use negative for backwards) in ticks
void drive(int distance) {

    configurePID(driveController, 0.5, 0, 0);
    targetPID(driveController, distance);

    SensorValue[LeftDrive] = 0;
    SensorValue[RightDrive] = 0;

    // Account for drift, which is about 600 ticks with this P loop
    distance -= 600;


    while(SensorValue[LeftDrive] < distance) {
        stepPID(driveController);

        driveController.value = SensorValue[LeftDrive];

        robot.leftDrive = driveController.output;
        robot.rightDrive = driveController.output;

        wait1Msec(20);
    }

    robot.leftDrive = 0;
    robot.rightDrive = 0;

}


PIDController turnController;

// Turns using gyro
void turn(float degrees) {
    configurePID(turnController, 0.3, 0, 0);

    while()
}