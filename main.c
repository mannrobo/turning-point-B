#pragma config(UART_Usage, UART1, uartVEXLCD, baudRate19200, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, in2,    powerExpander,  sensorAnalog)
#pragma config(Sensor, dgtl1,  flywheelRot,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  indexer,        sensorTouch)
#pragma config(Sensor, dgtl6,  leftDrive,      sensorQuadEncoder)
#pragma config(Sensor, dgtl8,  rightDrive,     sensorQuadEncoder)
#pragma config(Motor,  port1,           Uptake,        tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           FlywheelA,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           DriveFR,       tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port4,           DriveFL,       tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           CapFlipper,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           Intake,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           DriveBL,       tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           DriveBR,       tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           FlywheelB,     tmotorVex393_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/


// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

// Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

#include "lib\util.c"
#include "hal.c"

#include "lib\lcd.c"
#include "lib\motor.c"
#include "lib\pid.c"
#include "lib\auton.c"

void pre_auton() {
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;
  bDisplayCompetitionStatusOnLcd = false;

  SensorType[gyro] = sensorNone;
  lcdStartup();
  SensorType[gyro] = sensorGyro;
  wait1Msec(2000);

  // Clear flywheel Quad Encoder
  SensorValue[flywheelRot] = 0;

  // Clear Drive Encoders
  SensorValue[leftDrive] = 0;
  SensorValue[rightDrive] = 0;

  startTask(lcdDebug)

}

task autonomous() {

	startTask(hardwareAbstractionLayer);
  startTask(lcdDebug);

  // Turn on Flywheel, and wait for it to speed up
  targetTBH(robot.flywheel, 2400);
  while(robot.flywheel.process < 2400) {
    wait1Msec(20);
  }
  wait1Msec(400);

  // Turn on the uptakeA
  robot.uptake = REVERSE;
  wait1Msec(500);

  robot.leftDrive = 80;
  robot.rightDrive = 80;
  wait1Msec(50);
  robot.leftDrive = 0;
  robot.rightDrive = 0;

  // Turn to face the inside of the field
  // Positive is for blue, negative for red
  if(match.alliance == ALLIANCE_BLUE) {
    turn(100);
  } else {
    turn(-100);
  }

  robot.intake = REVERSE;
  robot.uptake = STOP;
  wait1Msec(200);

  // Wall sqaure (not distance so it happens for a short time)
  robot.leftDrive = -60;
  robot.rightDrive = -60;
  wait1Msec(400);

  // Set to low target
  targetTBH(robot.flywheel, 2000);
  robot.uptake = REVERSE;

  drive(1300);
  // Go backward for just a moment, to ensure the cap is flipped
  // And that the ball has been picked up
  drive(-1300);

if(match.alliance == ALLIANCE_BLUE) {
    turn(0);
  } else {
    turn(0);
  }

  // Shoot last ball

  robot.uptake = REVERSE;
  wait1Msec(2100);



  // // Alliance Park
  // drive(-900);


}

task usercontrol() {

  startTask(hardwareAbstractionLayer);
  startTask(lcdDebug);

  while (true) {
    wait1Msec(20);
  }

}
