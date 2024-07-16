#include "madgwick.h"

/*
 * @brief Retrieves the current time in milliseconds.
 * 
 * This function returns the current time in milliseconds since the program started. 
 * It uses the standard C library function `clock()` and converts the clock ticks to milliseconds.
 * 
 * @return The current time in milliseconds as an integer.
 */
static inline  int millis() {
    return (int)(clock() * 1000 / 100000);
}
static float previous_roll = 0.0f; // Used in unwrapping Roll value
/**
 * @brief Updates the orientation quaternion using the Madgwick filter algorithm.
 * 
 * This function reads the accelerometer and gyroscope data from the ASM330LHB sensor,
 * normalizes the accelerometer data, computes the gradient descent algorithm corrective step,
 * and integrates the rate of change of the quaternion over time. It also calculates the yaw
 * by integrating the gyroscope z-axis data(optionaly because without a magnetometer yaw get a lot of drift).
 * 
 * @param imu Pointer to the ASM330LHB sensor structure containing sensor data and configuration.
 */
void MadgwickQuaternionUpdate(ASM330LHB *imu){
    // Fetch imu data 
    float ax = imu->acc_ms2[0], ay = imu->acc_ms2[1], az = imu->acc_ms2[2], gx = imu->gyro_dps[0] *PI/180.0f, gy= imu->gyro_dps[1]*PI/180.0f,gz = imu->gyro_dps[2]*PI/180.0f;
    static float q[4];
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    float GyroMeasError = PI * (40.0f / 180.0f);        // gyroscope measurement error in rads/s (start at 40 deg/s)
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError;     // compute beta
    
    // Auxiliary variables to avoid repeated arithmetic
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _4q3 = 4.0f * q3;
    float _8q2 = 8.0f * q2;
    float _8q3 = 8.0f * q3;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;                                   // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Gradient decent algorithm corrective step
    s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
    s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q2 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + 4.0f * az;
    s3 = 4.0f * q1q1 * q3 + _2q1 * ax + 4.0f * q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3;
    s4 = 4.0f * q2q2 * q4 - _2q2 * ax + 4.0f * q3q3 * q4 - _2q3 * ay;
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);         // normalise step magnitude
    norm = 1.0f / norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Time between function calls
    static float currentTime = 0;
    float previousTime = currentTime;                           // Stored previous system time
    currentTime = millis();                                     // Gets actual system time
    float elapsedTime = (currentTime - previousTime) / 1000.0;  // Calculate the time interval between calls
    previousTime = currentTime;                                 // Updates previous time for the next call

    // Integrate to yield quaternion
    q1 += qDot1 * elapsedTime;
    q2 += qDot2 * elapsedTime;
    q3 += qDot3 * elapsedTime;
    q4 += qDot4 * elapsedTime;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);         // normalise quaternion
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

    static float yaw = 0;
    //yaw = ( yaw + (imu->gyro_dps[2] * elapsedTime*2.40)); //Integrate over time (%240 time offset due to drift)
    imu->yaw = yaw;
    imu->pitch  = (-asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * RAD_TO_DEG);
    float roll_raw  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * RAD_TO_DEG;
   // Unwrapping Roll Value (default: [-180, 180] Unwrapped: []
    float delta_roll = roll_raw - previous_roll;
    if (delta_roll > 180.0f) {
        roll_raw -= 360.0f;
    } else if (delta_roll < -180.0f) {
        roll_raw += 360.0f;
    }

    imu->roll = roll_raw -180;
    previous_roll = roll_raw;
    return;
}