/**
 * Abstract PID system, with specific implementations
 */

#pragma systemFile

typedef struct {

    float Kp; // Proportion. Main part of PID
    float Ki; // Integral.
    float Kd; // Derivative. Corrects for oscillation

    float target; // Target Value
    float value;  // Measured Value
    float error;

    float output; // Output of calculation

    float accumulatedError; // Used for Ki
    float lastError;  // Used for Kd

} PIDController;

void stepPID(PIDController & config) {
    config.error = config.target - config.value;

    if(config.Ki != 0) {
        config.accumulatedError += config.error;
    }

    config.output =
        (config.Kp * config.error) +
        (config.Ki * config.accumulatedError) +
        (config.Kd * config.lastError);

    config.lastError = config.error;
}

void setPID(PIDController & config, float value) {
    config.target = value;
}

void configurePID(PIDController & config, float Kp, float Ki, float Kd) {
    config.Kp = Kp;
    config.Ki = Ki;
    config.Kd = Kd;
}

void targetPID(PIDController & config, float target) {
    config.target = target;
}






/**
 * Specifics for a VELOCITY PID
 *
 * Note: this velocity pid works in ticks per second (not rpm)
 * User-facing functions will likely want to translate this to rpm
 */
typedef struct {

    PIDController controller;

    long lastTime;
    int deltaTime;

    int encoderPort;

    int deltaEncoder;
    int lastEncoder;

    float gearRatio; // Gear ratio between the encoder and the final output (2 means that for every tick of the encoder, the output goes 2 ticks)

} VelocityPID;

// Performs encoder measurements
void readEncoderVPID(VelocityPID & config) {
    config.deltaTime = nSysTime - config.lastTime;
    config.lastTime = nSysTime;

    config.deltaEncoder = SensorValue[config.encoderPort] - config.lastEncoder;
    config.lastEncoder = SensorValue[config.encoderPort];

    // Set controller in ticks per second
    config.controller.value = (config.deltaEncoder / config.deltaTime) * config.gearRatio;
}

void stepVPID(VelocityPID & config) {
    readEncoderVPID(config);
    stepPID(config.controller);
}

/**
 * Set a target for a velocity pid IN RPM, accounting for the gear ratio
 */
void targetVPID(VelocityPID & config, int target) {
    // Convert target to ticks per second
    config.controller.target = target / 60.0 / 1000.0 * 360.0;
}

/**
 * Returns the current speed of controller in RPM
 */
float measureVPID(VelocityPID & config) {
    return (config.controller.value * 1000.0) / 360.0 * 60.0;
}


/**
 * Tuning Utilies
 */

void graphVPID(VelocityPID & config, int series) {
    datalogAddValue(series, measureVPID(config));
}
