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

    SensorValue[leftDrive] = 0;
    SensorValue[rightDrive] = 0;

    // Account for drift, which is about 600 ticks with this P loop
    distance -= 600;


    while(SensorValue[leftDrive] < distance) {
        stepPID(driveController);

        driveController.value = SensorValue[leftDrive];

        robot.leftDrive = driveController.output;
        robot.rightDrive = driveController.output;

        wait1Msec(20);
    }

    robot.leftDrive = 60;
    robot.rightDrive = 60;

}


PIDController turnController;

// Turns using gyro
void turn(float degrees) {
    configurePID(turnController, 0.85, 0, 0);
    targetPID(turnController, degrees);

    do {
        turnController.value = SensorValue[gyro] / 10;
        stepPID(turnController);

        // Clamp turns even if the error is greater
        robot.leftDrive = -clamp(abs(turnController.output), 0, 70) * sgn(turnController.output);
        robot.rightDrive = clamp(abs(turnController.output), 0, 70) * sgn(turnController.output);

    } while(abs(turnController.error) > 5)
}
