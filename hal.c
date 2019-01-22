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
	int forward = abs(vexRT[Ch3]) > 32 ? vexRT[Ch3] : 0,
	turn = abs(vexRT[Ch4]) > 40 ? vexRT[Ch4] * 0.7 : 0;

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
		robot.indexerOverride = FORWARD;
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
	motorTarget[DriveBRA] = -robot.leftDrive
	motorTarget[DriveBRB] = -robot.rightDrive;
}

void flywheelStep() {
	// Firing Control
	robot.ballLoaded = SensorValue[ballDetector] <= 10;

	// Indexer allows best control of roller
	if(robot.firing) {
		robot.indexer = REVERSE;
		} else {
		robot.indexer = STOP;
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

		// Manual override of the indexer (in case of ball getting stuck)
		if(robot.indexerOverride != STOP) {
			robot.indexer = robot.indexerOverride;
		}

		switch(robot.indexer) {
		case FORWARD:
			motorTarget[Indexer] = 80;
			break;
		case REVERSE:
			motorTarget[Indexer] = -80;
			break;
		case STOP:
			motorTarget[Indexer] = 0;
			break;
		}




		motorControlStep();
		wait1Msec(20);
	}
}
