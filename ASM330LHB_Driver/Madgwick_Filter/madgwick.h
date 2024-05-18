#ifndef _MADGWICK_H__
#define _MADGWICK_H__

#include "../ASM330LHB/ASM330LHB.h"

/**
 * Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
 * (see https://x-io.co.uk/x-imu3/ for examples and more details)
 * which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
 * device orientation -- which can be converted to yaw, pitch, and roll.
*/
void MadgwickQuaternionUpdate(ASM330LHB *imu);

#endif  /* _MADGWICK_H__ */