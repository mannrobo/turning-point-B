/**
 * Hardware Abstraction Layer
 */

#include "lib\motor.c"
#include "lib\pid.c"
#include "lib\tbh.c"

enum motorMode {
    STOP = 0,
    FORWARD = 1,
    REVERSE = 2,
};


typedef struct {

    // Drive - Motor Values
    int leftDrive;
    int rightDrive;

    // Drive PIDs (actually just p-loops but whatever)
    PIDController driveController;
    PIDController turnController;

    // Flywheel
    TBHController flywheel;

    motorMode intake;
    motorMode uptake;

} HardwareAbstraction;


HardwareAbstraction robot;

/**
 * Individual subsystem control
 */
void controllerStep() {
    // Drive, with secret sauce!
    int forward = abs(vexRT[Ch3]) > 40 ? vexRT[Ch3] : 0,
        turn = abs(vexRT[Ch4]) > 40 ? vexRT[Ch4] * 0.7 : 0,
        left = forward + turn,
        right = forward - turn;

    robot.leftDrive = sgn(left) * rescaleTo(127, abs(left), abs(right), 0);
    robot.rightDrive = sgn(right) * rescaleTo(127, abs(left), abs(right), 1);

    // Intake
    if((vexRT[Btn5U] && robot.intake == FORWARD) || (vexRT[Btn5D] && robot.intake == REVERSE)) {
        robot.intake = STOP;
    } else if(vexRT[Btn5U]) {
        robot.intake = FORWARD;
    } else if (vexRT[Btn5D]) {
        robot.intake = REVERSE;
    }

    // Uptake
    if((vexRT[Btn6U] && robot.uptake == FORWARD) || (vexRT[Btn6D] && robot.uptake == REVERSE)) {
        robot.uptake = STOP;
    } else if(vexRT[Btn6U]) {
        robot.uptake = REVERSE;
    } else if (vexRT[Btn6D]) {
        robot.uptake = FORWARD;
    }

    // Flywheel (temporary)
    if(vexRT[Btn7U]) {
        targetTBH(robot.flywheel, 2500);
    } else if(vexRT[Btn7R]) {
        targetTBH(robot.flywheel, 1800);
    } else if(vexRT[Btn7D]) {
        targetTBH(robot.flywheel, 0);
    }

}

void driveStep() {
    motorTarget[DriveFL] = robot.leftDrive;
    motorTarget[DriveBL] = robot.leftDrive;

    motorTarget[DriveFR] = -robot.rightDrive;
    motorTarget[DriveBR] = -robot.rightDrive;
}

void flywheelStep() {
    calculateProcessTBH(robot.flywheel);
    stepTBH(robot.flywheel);

    motorTarget[FlywheelA] = robot.flywheel.output;
    motorTarget[FlywheelB] = robot.flywheel.output;
}

task hardwareAbstractionLayer() {
    initTBH(robot.flywheel, 0.0015, 2500, flywheelRot, 5.0);
    targetTBH(robot.flywheel, 0);

    while(true) {
        if(!bIfiAutonomousMode) {
           controllerStep();
        }
        driveStep();
        flywheelStep();

        // Misc.
        switch(robot.intake) {
            case FORWARD:
                motorTarget[Intake] = 127;
                break;
            case REVERSE:
                motorTarget[Intake] = -127;
                break;
            case STOP:
                motorTarget[Intake] = 0;
                break;
        }

        switch(robot.uptake) {
            case FORWARD:
                motorTarget[Uptake] = 127;
                break;
            case REVERSE:
                motorTarget[Uptake] = -127;
                break;
            case STOP:
                motorTarget[Uptake] = 0;
                break;
        }

        motorControlStep();
        wait1Msec(20);
    }
}
