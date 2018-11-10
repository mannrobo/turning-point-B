/**
 * Hardware Abstraction Layer
 */

#include "lib\motor.c"
#include "lib\pid.c"

typedef enum {
    STOP = 0,
    IN = 1,
    OUT = 2,
} intakeMode;

typedef struct {

    // Drive - Motor Values
    int leftDrive;
    int rightDrive;

    // Flywheel
    VelocityPID flywheel;

    // Intake
    intakeMode intake;

} HardwareAbstraction;


HardwareAbstraction robot;

/**
 * Individual subsystem control
 */
void controllerStep() {
    // Step 1: Drive
    int forward = abs(vexRT[Ch3]) > 60 ? vexRT[Ch3] : 0,
        turn = abs(vexRT[Ch4]) > 90 ? vexRT[Ch4] * 0.9 : 0,
        left = forward + turn,
        right = forward - turn;

    robot.leftDrive = sgn(left) * rescaleTo(127, abs(left), abs(right), 0);
    robot.rightDrive = sgn(right) * rescaleTo(127, abs(left), abs(right), 1);

    // Step 2: Intake
    if(vexRT[Btn5U]) {
        robot.intake = IN;
    } else if (vexRT[Btn5D]) {
        robot.intake = OUT;
    }
}

void driveStep() {
    motorTarget[DriveFL] = -robot.leftDrive;
    motorTarget[DriveBL] = -robot.leftDrive;

    motorTarget[DriveFR] = robot.rightDrive;
    motorTarget[DriveBR] = robot.rightDrive;
}

void flywheelStep() {
    stepVPID(robot.flywheel);

    // motorTarget[FlywheelA] = robot.flywheel.controller.output;
    // motorTarget[FlywheelB] = robot.flywheel.controller.output;
}

task hardwareAbstractionLayer() {
    configurePID(robot.flywheel.controller, 0, 0, 0);
    robot.flywheel.encoderPort = flywheelQuad;
    robot.flywheel.gearRatio = 5.0;

    while(true) {
        controllerStep();
        driveStep();
        flywheelStep();

        // Misc.
        switch(robot.intake) {
            case IN:
                motorTarget[Intake] = -127;
                break;
            case OUT:
                motorTarget[Intake] = 127;
                break;
            case STOP:
                motorTarget[Intake] = 0;
                break;
        }

        motorControlStep();
        wait1Msec(20);
    }
}
