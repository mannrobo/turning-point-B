/**
 * Take Back Half
 * Algorithm used for the flywgheel because PID tuning a flywheel is tough
 *
 * Inspirations:
 *  - https://www.chiefdelphi.com/forums/showpost.php?p=1247916
 *  - https://www.vexforum.com/index.php/14708-flywheel-velocity-control
 *  - https://www.vexforum.com/index.php/15558-tbh-tuning/0
 *  - https://www.vexforum.com/index.php/14654-team-929w-early-season-robot-reveal/p1#p133956
 *  - https://github.com/team751/2013RobotCode/blob/master/src/org/team751/speedcontrol/TakeBackHalfSpeedController.java
 *
 *
 */

typedef struct {

    // Target in RPM
    float setpoint;

    // Actual in RPM
    float process;

    // Gain (single tuning parameter!)
    float gain;

    // Error (setpoint - error)
    float error;

    // Error last frame
    float lastError;

    // Take Back Half
    float tbh;

    // Output
    float output;

    // RPM at Max Power (basically free spin * gear ration)
    float maxRPM;

    // For calculating the process
    float deltaTime;
    float lastTime;

    float deltaEncoder;
    float lastEncoder;

    // Encoder PORT (used to calculate RPM)
    int encoder;

    // Gear Ratio between the encoder and ouput
    float gearRatio;

} TBHController;

void initTBH(TBHController & controller, float gain, float maxRPM, int encoder, float gearRatio) {
    controller.gain = gain;
    controller.maxRPM = maxRPM;
    controller.lastError = 1;
    controller.encoder = encoder;
    controller.gearRatio = gearRatio;
}

void stepTBH(TBHController & controller) {

    // TBH responds weirdly to setting to zero, just let slew rate take care of it
    if(controller.setpoint == 0) {
        controller.output = 0;
    }


    // Calculate Error
    controller.error = controller.setpoint - controller.process;

    // Ramp up output
    controller.output += controller.gain * controller.error;

    // If the error has changed signs since last time, take back half
    if(sgn(controller.lastError) != sgn(controller.error)) {
        controller.output = 0.5 * (controller.output + controller.tbh);
        controller.tbh = controller.output;
    }

    controller.lastError = controller.error;

}

// Targets Controller
void targetTBH(TBHController & controller, float setpoint) {
    // Setting values to optimize spinup time

    if(controller.setpoint < setpoint) {
        controller.lastError = 1;
    } else if(controller.setpoint > setpoint) {
        controller.lastError = -1;
    }

    controller.tbh = (2 * setpoint / controller.maxRPM) - 1;
    controller.setpoint = setpoint;
}

// Calculates the process variable, specifically for RPM
void calculateProcessTBH(TBHController & controller) {
    controller.deltaTime = nSysTime - controller.lastTime;
    controller.lastTime = nSysTime;

    controller.deltaEncoder = SensorValue[controller.encoder] - controller.lastEncoder;
    controller.lastEncoder = SensorValue[controller.encoder];

    controller.process = ((float)controller.deltaEncoder / (float)controller.deltaTime) * 1000.0 *  controller.gearRatio / 360.0 * 60.0;
}
