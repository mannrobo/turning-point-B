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
	bool disableFlywheelControl;

	motorMode intake;
	motorMode indexer;

	motorMode descore;

	// Cap Flipper power
	int capFlipper;

	// Fire Button -- Robot should fire ball when ready
	bool firing;

	// Ball is ready to fire
	bool ballLoaded;

	// Double Shot (0 = inactive, 1 = before first shot, 2 = before second shot)
	int doubleShotMode;
	int resetCounter;

	// indexer override
	motorMode indexerOverride;

} HardwareAbstraction;


HardwareAbstraction robot;

void flywheelStep() {
	// Targeting
	if(vexRT[Btn7U]) {
		targetTBH(robot.flywheel, 2600);
	} else if(vexRT[Btn7R]) {
		targetTBH(robot.flywheel, 2400);
	} else if  (vexRT[Btn7L]) {
		targetTBH(robot.flywheel, 2500)
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
	}



	// Detect Balls for Firing Control
	robot.ballLoaded = SensorValue[ballDetector] <= 10;


	// Double Shot: Activate
	if (vexRT[Btn5D] && robot.doubleShotMode == 0 && robot.ballLoaded) {
		writeDebugStreamLine("Activate")
		robot.firing = true;
		robot.doubleShotMode = 1;
		robot.intake = REVERSE;
	}

	// Double Shot: Fired First Shot
	if (robot.doubleShotMode == 1 && abs(robot.flywheel.error) > 300) {
		robot.disableFlywheelControl = true;
		// Set to hold power for second shot
		if (robot.flywheel.setpoint == 2600) {
			robot.flywheel.output = 0;
		} else {
			robot.flywheel.output = 39;
		}

		robot.doubleShotMode = 2;
		robot.intake = REVERSE;
		robot.firing = false;
	}


	// Double Shot: Fire 2nd Ball
	if(robot.doubleShotMode == 2) {
		writeDebugStreamLine("Second Shot")
		robot.indexerOverride = FORWARD;
		robot.doubleShotMode = 3
		robot.resetCounter = nSysTime;
	}

	// Double Shot: Reset (via a timeout)
	if(robot.doubleShotMode == 3 && !robot.ballLoaded && nSysTime - robot.resetCounter > 2000) {
		writeDebugStreamLine("Reset")
		robot.indexerOverride = STOP;
		robot.intake = STOP;
		robot.doubleShotMode = 0;
		robot.disableFlywheelControl = false;
	}



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

	if (!robot.disableFlywheelControl) {
		stepTBH(robot.flywheel);
	}


	motorTarget[FlywheelOut] = robot.flywheel.output;
	// writeDebugStreamLine("%d", motorTarget[FlywheelOut])

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
	motorTarget[DriveBLB] = robot.leftDrive;

	motorTarget[DriveFR] = -robot.rightDrive;
	motorTarget[DriveBRB] = -robot.rightDrive;
}


void takerStep() {

	if(vexRT[Btn6U])  {
		robot.intake = REVERSE;
	} else if (vexRT[Btn6D]) {
		robot.intake = FORWARD;
	} else if (!bIfiAutonomousMode) {
		robot.intake = STOP;
	}

	// Shoot out ball if required
	if(robot.indexerOverride != STOP) {
		robot.indexer = robot.indexerOverride;
	}

	// If fire mode, then move indexer to catch balls
	if(robot.firing) {
		robot.intake = REVERSE;
	}

	// Descore
	if (vexRT[Btn8L]) {
		motorTarget[DescoreL] = 127;
		motorTarget[DescoreR] = 127;
	} else if (vexRT[Btn8R]) {
		motorTarget[DescoreL] = -127;
		motorTarget[DescoreR] = -127;
	} else {
		motorTarget[DescoreL] = 0;
		motorTarget[DescoreR] = 0;
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
		takerStep();
		flywheelStep();
		motorControlStep();
		wait1Msec(20);
	}
}
