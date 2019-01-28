/**
* Hardware Abstraction Layer
*/

#include "lib\motor.c"
#include "lib\pid.c"
#include "lib\tbh.c"
#include "lib\util.c"

enum motorMode {
	STOP = 0,
	FORWARD = 1,
	REVERSE = 2,
};


typedef struct {

	// Drive - Motor Values
	int leftDrive;
	int rightDrive;

	// Drive Directional Control
	int forward;
	int turn;

	// Drive PIDs (actually just p-loops but whatever)
	PIDController driveController;
	PIDController turnController;

	// Flywheel
	TBHController flywheel;

	motorMode intake;
	motorMode indexer;

	// Cap Flipper power
	int capFlipper;

	// Fire Button -- Robot should fire ball when ready
	bool firing;

	// Ball is ready to fire
	bool ballLoaded;

	// indexer override
	motorMode indexerOverride;

} HardwareAbstraction;


HardwareAbstraction robot;

/**
* Individual subsystem control
*/
void controllerStep() {
	// Arcade Drive
	int forward = logistic(vexRT[Ch3]),
		turn = logistic(vexRT[Ch4]);


	robot.forward = forward;
	robot.turn = turn;


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

	// Manual Flywheel Control
	if(vexRT[Btn8U] && robot.flywheel.setpoint < robot.flywheel.maxRPM) {
		targetTBH(robot.flywheel, robot.flywheel.setpoint + 10);
		} else if (vexRT[Btn8D] && robot.flywheel.setpoint > 0) {
		targetTBH(robot.flywheel, robot.flywheel.setpoint - 10);
	}

	// Fire Control
	if(vexRT[Btn5U]) {
		robot.firing = true;
		} else if(vexRT[Btn5D]) {
		robot.indexerOverride = REVERSE;
		} else {
		robot.indexerOverride = STOP;
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
    // Note: Back A motors are INTENTIONALLY reversed
	motorTarget[DriveFL] = robot.leftDrive;
	motorTarget[DriveBLA] = robot.rightDrive;
	motorTarget[DriveBLB] = robot.leftDrive;

	motorTarget[DriveFR] = -robot.rightDrive;
	motorTarget[DriveBRA] = -robot.leftDrive;
	motorTarget[DriveBRB] = -robot.rightDrive;
}

void flywheelStep() {
	// Firing Control
	robot.ballLoaded = SensorValue[ballDetector] <= 15;

	// When to fire: if a ball is loaded, the flywheel error is sufficently small, and the flywheel speed is above a threshold
	if(robot.ballLoaded && robot.flywheel.error < 100 && robot.flywheel.setpoint > 1000 && robot.firing) {
		robot.indexer = FORWARD;
	// Hold balls and prepare to fire
	} else if(robot.ballLoaded) {
		robot.indexer = STOP;
	// Otherwise try to catch balls
	} else {
		robot.indexer = FORWARD;
		robot.firing = false;
	}

	// Flywheel Itself
	calculateProcessTBH(robot.flywheel);
	stepTBH(robot.flywheel);

	motorTarget[FlywheelOut] = robot.flywheel.output;
}

task hardwareAbstractionLayer() {
	initTBH(robot.flywheel, 0.0015, 3500, flywheel, 5.0);
	targetTBH(robot.flywheel, 0);

	while(true) {
		if(!bIfiAutonomousMode) {
			controllerStep();
		}
		driveStep();
		flywheelStep();


		// Shoot out ball if required
		if(robot.indexerOverride != STOP) {
			robot.indexer = robot.indexerOverride;
			robot.intake = robot.indexerOverride;
		}

		switch(robot.indexer) {
		case FORWARD:
			motorTarget[Indexer] = 70;
			break;
		case REVERSE:
			motorTarget[Indexer] = -70;
			break;
		case STOP:
			motorTarget[Indexer] = 0;
			break;
		}

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



		motorControlStep();
		wait1Msec(20);
	}
}
