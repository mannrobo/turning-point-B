/**
 * util.c - General purpose utility functions, like those that might be found in C's standard library (but are missing in RobotC, grr!)
 */

#pragma systemFile

#define arraySize(a) (sizeof(a)/sizeof(a[0]))
#define PI 3.1415926535

/**
 * Performs modulus for floats
 * @param float x The numerator
 * @param float y The demominator
 * @return float The remainder of x / y
 */
float fmodf(float x, float y) {
  return (x - y * floor(x / y));
}

/**
 * Clamps a value to a maximum and minimum
 * @param float val The value to clamp
 * @param float min The minimum value for the value
 * @param float max The maximum value for the value
 * @return float The clamped value
 */
float clamp(float val, float min, float max) {
  return val > max ? max : val < min ? min : val;
}

/**
 * Corrects a single direction encoder, giving it the corrected value, based on the specified associated motor
 * @param int lastValue The last returned value of this function
 * @param int measuredValue The new measured value of the sensor
 * @param int pwm The pwm of the motor the sensor is attached to
 * @return int The correct position of the encoder
 * EXAMPLE:
 *
 * int encoderValue = SensorValue[encoder];
 * while(true) {
 *  encoderValue = encoderDirect(encoderValue, SensorValue[encoder], motor[port3]);
 *  wait1Msec(20);
 * }
 *
 * NOTE: This function will not be nearly as accurate as merely using a correct
 * 2 direction encoder, it is meant as a backup only!
 */
int encoderDirect(int lastValue, int measuredValue, int pwm) {
  return lastValue + sgn(pwm) * (measuredValue - lastValue);
}

/**
 * Rescales two values to a specified maximum and returns the specified value
 * @param int max The maximum value, used to calculate the scalar
 * @param int alpha The first value to rescale
 * @param int beta The second value to rescale
 * @param int side The value to return, 0 for alpha, 1 for beta
 **/
int rescaleTo(int max, int alpha, int beta, int side) {
  if (alpha <= max && beta <= max) return side ? beta : alpha; // If no sclaing is needed, return

  float scalar = alpha > beta ? (float) max / (float) alpha : (float) max / (float) beta;
  return (int)(side ? beta * scalar : alpha * scalar);
}




#define STRTOK_MAX_TOKEN_SIZE 20
#define STRTOK_MAX_BUFFER_SIZE 50

/**
 * Tokenise an array of chars, using a seperator
 * @param buffer pointer to buffer we're parsing
 * @param token pointer to buffer to hold the tokens as we find them
 * @param seperator the seperator used between tokens
 * @return true if there are still tokens left, false if we're done
 */
bool strtok(char *buffer, char *token, char *seperator)
{
  int pos = stringFind(buffer, seperator);
  char t_buff[STRTOK_MAX_BUFFER_SIZE];

  // Make sure we zero out the buffer and token
  memset(token, 0, STRTOK_MAX_TOKEN_SIZE);
  memset(&t_buff[0], 0, STRTOK_MAX_BUFFER_SIZE);

  // Looks like we found a seperator
  if (pos >= 0)
  {
    // Copy the first token into the token buffer, only if the token is
    // not an empty one
    if (pos > 0)
      memcpy(token, buffer, pos);
    // Now copy characters -after- the seperator into the temp buffer
    memcpy(&t_buff[0], buffer+(pos+1), strlen(buffer) - pos);
    // Zero out the real buffer
    memset(buffer, 0, strlen(buffer) + 1);
    // Copy the temp buffer, which now only contains everything after the previous
    // token into the buffer for the next round.
    memcpy(buffer, &t_buff[0], strlen(&t_buff[0]));
    return true;
  }
  // We found no seperator but the buffer still contains a string
  // This can happen when there is no trailing seperator
  else if(strlen(buffer) > 0)
  {
    // Copy the token into the token buffer
    memcpy(token, buffer, strlen(buffer));
    // Zero out the remainder of the buffer
    memset(buffer, 0, strlen(buffer) + 1);
    return true;
  }
  return false;
}
