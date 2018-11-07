/**
 * Abstract PID system, with specific implementations
 */

enum PIDType

typedef struct {

    float Kp; // Proportion. Main part of PID
    float Ki; // Integral. 
    float Kd; // Derivative. Corrects for oscillation

    float target; // Target Value
    float value;  // Measured Value

    float setPoint; // Output of calculation

    float accumulatedError; // Used for Ki
    float lastError;  // Used for Kd

} PIDController;

void stepPID(PIDController & config) {
    float error = config.target - config.value;

    if(config.Ki != 0) {
        config.accumulatedError += error;
    }

    config.setPoint =
        (config.Kp * error) +
        (config.Ki * config.accumulatedError) +
        (config.Kd * config.lastError);

    config.lastError = error;
}

void setPID(PIDController & config, float value) {
    config.target = value;
}

void configurePID(PIDController & config, float Kp, float Ki, float Kd) {
    config.Kp = Kp;
    config.Ki = Ki;
    config.Kd = Kd;
}

void tunePID(PIDController & config, int tuneConstant, int sign) {
    if(tuneConstant == 0) {
        config.Kp += sign * 0.05
    } else if (tuneConstant == 1) {
        config.Ki += sign * 0.01
    } else {
        config.Kd += sign * 0.01
    }
}

/**
 * Specifics for a VELOCITY PID
 */
