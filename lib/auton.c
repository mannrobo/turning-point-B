/**
 * Utilies for autonomous programs
 */

#include "../hal.c"
#include "pid.c"



// Drives a specific distance (forward, use negative for backwards) in ticks
void drive(int distance) {

    configurePID(robot.driveController, 0.5, 0, 0);
    targetPID(robot.driveController, distance);

    SensorValue[leftDrive] = 0;
    SensorValue[rightDrive] = 0;

    // Account for drift, which is about 600 ticks with this P loop
    distance -= 600;


    while(SensorValue[leftDrive] < distance) {
        stepPID(robot.driveController);

        robot.driveController.value = SensorValue[leftDrive];

        robot.leftDrive = robot.driveController.output;
        robot.rightDrive = robot.driveController.output;

        wait1Msec(20);
    }

    robot.leftDrive = 60;
    robot.rightDrive = 60;

}


// Turns using gyro
void turn(float degrees) {
    configurePID(robot.turnController, 0.85, 0, 0);
    targetPID(robot.turnController, degrees);

    do {
        robot.turnController.value = SensorValue[gyro] / 10;
        stepPID(robot.turnController);

        // Clamp turns even if the error is greater
        robot.leftDrive = -clamp(abs(robot.turnController.output), 0, 70) * sgn(robot.turnController.output);
        robot.rightDrive = clamp(abs(robot.turnController.output), 0, 70) * sgn(robot.turnController.output);

    } while(abs(robot.turnController.error) > 5)
}
