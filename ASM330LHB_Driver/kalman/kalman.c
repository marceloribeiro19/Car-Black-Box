#include "kalman.h"

const float A = 1;
const float B = 0;
const float C = 0;
const float D = 0;
const float u = 0;
const float H = 1;
const float Q = 0.1;
const float R = 0.0001;
const float I = 1;

float Kalman_Roll(float rawRoll, ASM330LHB *imu){
    //Prediction
    float Xapriori = A * imu->Xant_Roll + B * u;
    float Papriori = A * imu->Pant_Roll * A + Q;
    //Correction
    float K = Papriori * H / (H * Papriori * H + R);
    float X = Xapriori + K * (rawRoll - H * Xapriori);
    float P = (I - K * H) * Papriori;
    imu->Pant_Roll = P;
    imu->Xant_Roll = X;
    return imu->Xant_Roll;
}

float Kalman_Pitch(float rawPitch, ASM330LHB *imu){
    //Prediction
    float Xapriori = A * imu->Xant_Pitch + B * u;
    float Papriori = A * imu->Pant_Pitch * A + Q;
    //Correction
    float K = Papriori * H / (H * Papriori * H + R);
    float X = Xapriori + K * (rawPitch - H * Xapriori);
    float P = (I - K * H) * Papriori;
    imu->Pant_Pitch = P;
    imu->Xant_Pitch = X;
    return imu->Xant_Pitch;
}

float Kalman_Yaw(float rawYaw, ASM330LHB *imu){
    //Prediction
    float Xapriori = A * imu->Xant_Yaw + B * u;
    float Papriori = A * imu->Pant_Yaw * A + Q;
    //Correction
    float K = Papriori * H / (H * Papriori * H + R);
    float X = Xapriori + K * (rawYaw - H * Xapriori);
    float P = (I - K * H) * Papriori;
    imu->Pant_Yaw = P;
    imu->Xant_Yaw = X;
    return imu->Xant_Yaw;
}