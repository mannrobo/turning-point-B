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
	REVERSE = 2
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

	bool doubleShotActive;

	// indexer override
	motorMode indexerOverride;

} HardwareAbstraction;


HardwareAbstraction robot;

void flywheelStep() {
	// Targeting
	if(vexRT[Btn7U]) {
		targetTBH(robot.flywheel, 2500);
	} else if(vexRT[Btn7R]) {
		targetTBH(robot.flywheel, 2400);
	} else if(vexRT[Btn7D]) {
		targetTBH(robot.flywheel, 0);
	}

	// Manual adjust
	if(vexRT[Btn8U] && robot.flywheel.setpoint < robot.flywheel.maxRPM) {
		robot.flywheel.setpoint += 10;
	} else if (vexRT[Btn8D] && robot.flywheel.setpoint > 0) {
		robot.flywheel.setpoint -= 10;
	}

	// Firing Control
	if(vexRT[Btn5U]) {
		robot.firing = true;
	} else if(vexRT[Btn5D]) {
		robot.indexerOverride = FORWARD;
	} else {
		robot.indexerOverride = STOP;
	}



	// Detect Balls for Firing Control
	robot.ballLoaded = SensorValue[ballDetector] <= 10;


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

void driveStep() {
	// Arcade Drive
	if(!bIfiAutonomousMode) {
		int forward = logistic(vexRT[Ch3]),
			turn = logistic(vexRT[Ch4]);


		robot.forward = forward;
		robot.turn = turn;


		robot.leftDrive = forward + turn;
		robot.rightDrive = forward - turn;
	}

    // Note: Back A motors are INTENTIONALLY reversed
	motorTarget[DriveFL] = robot.leftDrive;
	motorTarget[DriveBLA] = robot.rightDrive;
	motorTarget[DriveBLB] = robot.leftDrive;

	motorTarget[DriveFR] = -robot.rightDrive;
	motorTarget[DriveBRA] = -robot.leftDrive;
	motorTarget[DriveBRB] = -robot.rightDrive;
}


void takerStep() {

	if(vexRT[Btn6U] && vexRT[Btn6D]) {
		robot.intake = STOP;
	} else if(vexRT[Btn6U])  {
		robot.intake = REVERSE;
	} else if (vexRT[Btn6D]) {
		robot.intake = FORWARD;
	}

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
}

task hardwareAbstractionLayer() {
	initTBH(robot.flywheel, 0.0015, 3500, flywheel, 5.0);
	targetTBH(robot.flywheel, 0);

	while(true) {
		driveStep();
		flywheelStep();
		takerStep();
		motorControlStep();
		wait1Msec(20);
	}
}
