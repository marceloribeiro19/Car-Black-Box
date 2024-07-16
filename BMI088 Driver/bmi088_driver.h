#ifndef __BMI088_H__
#define __BMI088_H__

#include "xiicps.h"              
#include <xscugic.h>
#include "xparameters.h"
#include <xiicps_hw.h>
#include <xil_printf.h>
#include <stdint.h>
#include <stdio.h>
#include "sleep.h"
#include <math.h>
#include "xil_exception.h"
#include "xinterrupt_wrap.h"

/*
 * CONSTANTS
 */
static const float G   = 9.81f;
static const float pi  = 3.14159265358f;
static const float D2R = pi / 180.0f;
static const float RAD_TO_DEG	= 57.2957795131;
static const float DEG_TO_RAD	= 0.0174532925;
static const float PI           = 3.141592;
/*
 *  CONSTANTS
 */
static const u16 IIC_DEVICE_ID = 0;
static const u32 I2C_00_BASE_ADDR = 0xe0004000;
static const int IIC_CLK_RATE  = 400000; //300kHz

// transformation from sensor frame to right hand coordinate system
static const int16_t tX[3] = {1, 0, 0};
static const int16_t tY[3] = {0, -1, 0};
static const int16_t tZ[3] = {0, 0, -1};


/*
 * CONFIGURATION REGISTERS
 */
static const u8 BMI088_ACC_ADDR           = 0x18;
static const u8 BMI088_ACC_SOFTRESET_ADDR   = 0x7E;
static const u8 BMI088_ACC_PWR_CTRL         = 0x7D;
static const u8 BMI088_ACC_PWR_CONF         = 0x7C;
static const u8 BMI088_ACC_RANGE            = 0x41;
static const u8 BMI088_ACC_CONF             = 0x40;
static const u8 BMI088_ACC_ON               = 0x04;
static const u8 BMI088_ACC_OFF              = 0x00;
static const u8 BMI088_ACC_CHIP_ID_REG      = 0x00;


static const u8 BMI088_GYRO_ADDR            = 0x68;
static const u8 BMI088_GYRO_SOFTRESET_ADDR  = 0x14;
static const u8 BMI088_GYRO_RANGE           = 0x0F;
static const u8 BMI088_GYRO_BW              = 0x10;
static const u8 BMI088_GYRO_LPM1            = 0x11;
static const u8 GYRO_SELF_TEST_ADDR         = 0x3C;
static const u8 BMI088_GYRO_CHIP_ID_REG     = 0x00;


/*
 * Ranges
 */
static const u8 ACC_RANGE_3G           = (3.0f * 9.81f);  // +/- 3g
static const u8 ACC_RANGE_6G           = (6.0f * 9.81f);  // DEFAULT +/- 6g
static const u8 ACC_RANGE_12G          = (12.0f * 9.81f);  // +/- 12g
//static const float GYRO_RANGE_2000_DPS = 2000.0f; 
static const float GYRO_RANGE_RADS     = 2000.0f * D2R; 
static const u8 GYPRO_RANGE_SET_500DPS = 0x02;


/*
 * OUTPUT REGISTERS
 */
static const u8 BMI088_ACC_DATA_START   = 0x12;
static const u8 BMI088_GYRO_DATA_START  = 0x02;
static const u8 BMI088_TEMP_MSB_REG     = 0x22;

/* 
 * COMMANDS
 */
static u8 ACC_ENABLE_CMD      = 0x04;
static u8 GYRO_SOFT_RESET_CMD = 0xB6;
static u8 ACC_CONF            = 0xA9;
/*
 * ODR (OUTPUT DATA RATE)
 */
static const u8 ACC_ODR_100HZ  = 0xA8;  // ODR = 100Hz, bandwidth = 32Hz
static const u8 GYRO_ODR_200HZ = 0x02;  // ODR = 200Hz, bandwidth = 100Hz


typedef struct{
    //Chip ids
    u8 ACC_CHIP;
    u8 GYRO_CHIP;

    //Output data rates
    int8_t ACC_ODR;
    int8_t GYRO_ODR; 

    //Data
    u8 acc_data[3];         //raw
    float acc_ms2[3];     //m/s²

    float gyro_dps[3];
    float gyro_rads[3];

    /* KALMAN FILTER */
    volatile float Xant_Roll, Xant_Pitch, Xant_Yaw;
	volatile float Pant_Roll, Pant_Pitch, Pant_Yaw;

    float temperature;
    float roll, pitch, yaw;
    XIicPs_Config *Config;
} BMI088;

int BMI088_Init(BMI088 *imu);
int BMI088_Process(BMI088* imu);
int BMI088_readTEMP(BMI088 *imu);


/*
 * The following counters are used to determine when the entire buffer has
 * been sent and received.
 */
#define IIC_INTR_ID             XPAR_XIICPS_0_INTERRUPTS
extern volatile u32 SendComplete;
extern volatile u32 RecvComplete;
extern volatile u32 TotalErrorCount;

#endif