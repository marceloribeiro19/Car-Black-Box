
#ifndef _KALMAN_H__
#define _KALMAN_H__

#include "../ASM330LHB/ASM330LHB.h"



float Kalman_Roll(float rawRoll, ASM330LHB *imu);
float Kalman_Pitch(float rawPitch, ASM330LHB *imu);
float Kalman_Yaw(float rawYaw, ASM330LHB *imu);
#endif  /* _KALMAN_H__ */