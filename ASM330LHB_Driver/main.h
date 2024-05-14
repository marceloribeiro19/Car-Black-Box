#pragma once

#include <stdio.h>
#include <unistd.h> // Para sleep()
#include <pthread.h>
#include "ASM330LHB/ASM330LHB.h"

#define STORAGE_PATH    "/IMU/BlackBox.txt"
#define SAMPLE_TIME     0.00518f     //This time is the real time mesured between samples in seconds (5.18ms = 66bytes tranfered)
#define SIZE_TEST       127413       //Bytes ->Corresponds to 10 seconds of data (Replace by size_test and verify the file clean at 10 seconds)
#define MAX_SIZE        16000000     //Bytes -> 16MB of space corresponding to 21min of IMU data
#define NUM_THREADS     3

#define MODE REPLAY  //REPLAY

static __u8 ASM330LHB_uartTransmit(ASM330LHB *imu, char* message);
static __u8 ASM330LHB_uartInit(ASM330LHB *imu);

typedef enum{
    REAL_TIME,
    REPLAY     //no replay o calculo do roll e do pitch tem de parar
   // OFF
}operatingMode;

typedef struct{
    operatingMode mode;
    ASM330LHB imu;
    pthread_mutex_t mutex;
}IMUData;
