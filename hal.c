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

    // Cap Flipper power
    int capFlipper;

    // Fire Button -- Robot should fire ball when ready
    bool firing;

    // Ball is ready to fire
    bool ballLoaded;

    // Uptake override
    motorMode uptakeOverride;

} HardwareAbstraction;


HardwareAbstraction robot;

/**
 * Individual subsystem control
 */
void controllerStep() {
    // Arcade Drive
    int forward = abs(vexRT[Ch3]) > 32 ? vexRT[Ch3] : 0,
        turn = abs(vexRT[Ch4]) > 40 ? vexRT[Ch4] * 0.7 : 0;

    robot.leftDrive = forward + turn;
    robot.rightDrive = forward - turn;

    // Intake
    if(vexRT[Btn6U]) {
        robot.intake = REVERSE;
    } else if (vexRT[Btn6D]) {
        robot.intake = FORWARD;
    }

    // Flywheel
    if(vexRT[Btn7U]) {
        targetTBH(robot.flywheel, 2500);
    } else if(vexRT[Btn7R]) {
        targetTBH(robot.flywheel, 1800);
    } else if(vexRT[Btn7D]) {
        targetTBH(robot.flywheel, 0);
    }

    // Fire Control
    if(vexRT[Btn5U]) {
        robot.firing = true;
    } else if(vexRT[Btn5D]) {
        robot.uptakeOverride = FORWARD;
    } else {
        robot.uptakeOverride = STOP;
    }

    // Cap Flipper
    if(vexRT[Btn8U]) {
        robot.capFlipper = -80;
    } else if(vexRT[Btn8D]) {
        robot.capFlipper = 80;
    } else {
        robot.capFlipper = 0;
    }
}

void driveStep() {
    motorTarget[DriveFL] = robot.leftDrive;
    motorTarget[DriveBL] = robot.leftDrive;

    motorTarget[DriveFR] = -robot.rightDrive;
    motorTarget[DriveBR] = -robot.rightDrive;
}

void flywheelStep() {
    // Firing Control
    robot.ballLoaded = SensorValue[ballDetector] <= 8;

    // If we've fired the ball, reset
    if(!robot.ballLoaded && robot.firing) {
        robot.firing = false;
    // Fire the ball if the conditions are met (loaded, flywheel's right, fire command)
    } else if(robot.ballLoaded &&
       robot.firing &&
       abs(robot.flywheel.error) < 250
    ) {
        robot.uptake = REVERSE;
    // If we're not firing, move the balls into firing position
    } else if(!robot.ballLoaded && !robot.firing) {
        robot.uptake = REVERSE;
        // Stop the firing to prevent accidental shooting
    } else {
        robot.uptake = STOP;
    }


    // Flywheel Itself
    calculateProcessTBH(robot.flywheel);
    stepTBH(robot.flywheel);

    motorTarget[FlywheelA] = robot.flywheel.output;
    motorTarget[FlywheelB] = robot.flywheel.output;
}

task hardwareAbstractionLayer() {
    initTBH(robot.flywheel, 0.0015, 3500, flywheelRot, 5.0);
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

        // Manual override of the uptake (in case of ball getting stuck)
        if(robot.uptakeOverride != STOP) {
            robot.uptake = robot.uptakeOverride;
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



        motorTarget[CapFlipper] = robot.capFlipper;

        motorControlStep();
        wait1Msec(20);
    }
}
