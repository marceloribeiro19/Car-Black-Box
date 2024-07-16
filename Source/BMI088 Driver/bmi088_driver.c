#include "bmi088_driver.h"
#include "../Madgwick/madgwick.h"
#include "BMI088_Kalman/BMI088_kalman.h"
#include <sleep.h>

static XIicPs Iic_Instance;

/****************************PRIVATE FUNCTIONS****************************/
/***I2C READ WRITE OPERATIONS***/

/**
 * @brief Read data from a register of the BMI088 sensor.
 *
 * This function reads data from a register of the BMI088 sensor via I2C communication.
 *
 * @param imu Pointer to the BMI088 structure.
 * @param Address I2C address of the sensor.
 * @param Reg Register address to read from.
 * @param Buffer Pointer to the buffer to store the read data.
 * @param ByteCount Number of bytes to read.
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */

static int BMI088_readReg(BMI088 *imu, u16 Address, u8 Reg, u8 *Buffer, s32 ByteCount) {
    int Status;

    Status = XIicPs_MasterSendPolled(&Iic_Instance, &Reg, 1, Address);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    Status = XIicPs_MasterRecvPolled(&Iic_Instance, Buffer, ByteCount, Address);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/**
 * @brief Write data to a register of the BMI088 sensor.
 *
 * This function writes data to a register of the BMI088 sensor via I2C communication.
 *
 * @param imu Pointer to the BMI088 structure.
 * @param Address I2C address of the sensor.
 * @param Reg Register address to write to.
 * @param Data Pointer to the data buffer to write.
 * @param ByteCount Number of bytes to write.
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */
 
static int BMI088_writeReg(BMI088 *imu, u16 Address, u8 Reg, u8 *Data, s32 ByteCount) {
    int Status;
    u8 SendBuffer[64];

    SendBuffer[0] = Reg;
    for (int i = 0; i < ByteCount; i++) {
        SendBuffer[i + 1] = Data[i];
    }

    Status = XIicPs_MasterSendPolled(&Iic_Instance, SendBuffer, ByteCount + 1, Address);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/********INIT FUNCTIONS********/

/**
 * @brief Initialize the BMI088 sensor with default settings.
 *
 * This function initializes the BMI088 sensor with default settings.
 *
 * @param imu Pointer to the BMI088 structure.
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */
static int BMI088_Init_I2C(BMI088 *imu) {
    int Status;

    printf("Initializing I2C...\n");
    imu->Config = XIicPs_LookupConfig(IIC_DEVICE_ID);
    if (imu->Config == NULL) {
        printf("Erro finding I2C config !\n");
        return XST_FAILURE;
    }

    Status = XIicPs_CfgInitialize(&Iic_Instance, imu->Config, I2C_00_BASE_ADDR);
    if (Status != XST_SUCCESS) {
        printf("Error initializing I2C !\n");
        return XST_FAILURE;
    }

    Status = XIicPs_SelfTest(&Iic_Instance);
    if (Status != XST_SUCCESS) {
        printf("Error in I2C self test !\n");
        return XST_FAILURE;
    }

    Status = XIicPs_SetSClk(&Iic_Instance, IIC_CLK_RATE);  //I2C CLOCK
    if (Status != XST_SUCCESS) {
        printf("Error defining I2C clock !\n");
        return XST_FAILURE;
    }

    usleep(30000);
    printf("I2C initialized with success !\n");

    return XST_SUCCESS;
}

/**
 * @brief Initialize the gyroscope of the BMI088 sensor.
 *
 * This function initializes the gyroscope of the BMI088 sensor.
 *
 * @param imu Pointer to the BMI088 structure.
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */
static int BMI088_Init_GYRO(BMI088 *imu){
    int Status;

    Status = BMI088_readReg(imu, BMI088_GYRO_ADDR, BMI088_GYRO_CHIP_ID_REG, &imu->GYRO_CHIP, 1);
    if ((Status != XST_SUCCESS) || (imu->GYRO_CHIP != 0x0F)) {
        xil_printf("Error reading gyroscope chip id! \n\r");
        return XST_FAILURE;
    }

    xil_printf("Gyroscope ON! \n\r");

    return XST_SUCCESS;
}

/**
 * @brief Initialize the accelerometer of the BMI088 sensor.
 *
 * This function initializes the accelerometer of the BMI088 sensor.
 *
 * @param imu Pointer to the BMI088 structure.
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */
static int BMI088_Init_ACC(BMI088 *imu){
    int Status;

    Status = BMI088_readReg(imu, BMI088_ACC_ADDR, BMI088_ACC_CHIP_ID_REG, &imu->ACC_CHIP, 1);
    if ((Status != XST_SUCCESS) || (imu->ACC_CHIP != 0x1E) ) {
        xil_printf("Error reading accelerometer chip id! \n\r");
        return XST_FAILURE;
    }

    xil_printf("Accelerometer chip id: 0x%X \n\r", imu->ACC_CHIP);

    Status = BMI088_writeReg(imu, BMI088_ACC_ADDR, BMI088_ACC_PWR_CTRL, &ACC_ENABLE_CMD, 1);    //Enables ACC
    if (Status != XST_SUCCESS) {
        xil_printf("Error turning on Accelerometer! \n\r");
        return XST_FAILURE;
    }

    xil_printf("Accelerometer ON!\n\r");

    return XST_SUCCESS;
}

/*******DATA ACQUISITION*******/

/**
 * @brief Read gyroscope data from the BMI088 sensor.
 *
 * This function reads gyroscope data from the BMI088 sensor.
 *
 * @param imu Pointer to the BMI088 structure.
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */
static int BMI088_readGYRO(BMI088 *imu) {
    int Status;
    u8 gyro_data[6];
    int16_t gyro[3];

    Status = BMI088_readReg(imu, BMI088_GYRO_ADDR, BMI088_GYRO_DATA_START, gyro_data, 6);
    if (Status != XST_SUCCESS) {
        printf("Error reading data from the gyroscope! \n\r");
        return XST_FAILURE;
    }

    gyro[0] = ((gyro_data[1] << 8) | gyro_data[0]);  // X-axis
    gyro[1] = ((gyro_data[3] << 8) | gyro_data[2]);  // Y-axis
    gyro[2] = ((gyro_data[5] << 8) | gyro_data[4]);  // Z-axis


    imu->gyro_dps[0] = (float) (gyro[0] * tX[0] + gyro[1] * tX[1] + gyro[2] * tX[2])  / 32768.0f * 2000;
    imu->gyro_dps[1] = (float) (gyro[0] * tY[0] + gyro[1] * tY[1] + gyro[2] * tY[2])  / 32768.0f * 2000;
    imu->gyro_dps[2] = (float) (gyro[0] * tZ[0] + gyro[1] * tZ[1] + gyro[2] * tZ[2])  / 32768.0f * 2000;

    usleep(10000);
    return XST_SUCCESS;
}

/**
 * @brief Read accelerometer data from the BMI088 sensor.
 *
 * This function reads accelerometer data from the BMI088 sensor.
 *
 * @param imu Pointer to the BMI088 structure.
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */
static int BMI088_readACC(BMI088 *imu) {
    int Status;
    u8 accel_data[6];
    int16_t accel[3];

    Status = BMI088_readReg(imu, BMI088_ACC_ADDR, BMI088_ACC_DATA_START, accel_data, 6);
    if (Status != XST_SUCCESS) {
        printf("Error reading data from accelerometer! \n\r");
        return XST_FAILURE;
    }

    // Combine bytes and perform sign extension for each axis
    accel[0] = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    accel[1] = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    accel[2] = (int16_t)((accel_data[5] << 8) | accel_data[4]);

    imu->acc_ms2[0] = (float) (accel[0] * tX[0] + accel[1] * tX[1] + accel[2] * tX[2]) * (ACC_RANGE_3G / 32768.0f);
    imu->acc_ms2[1] = (float) (accel[0] * tY[0] + accel[1] * tY[1] + accel[2] * tY[2]) * (ACC_RANGE_3G / 32768.0f);
    imu->acc_ms2[2] = (float) (accel[0] * tZ[0] + accel[1] * tZ[1] + accel[2] * tZ[2]) * (ACC_RANGE_3G / 32768.0f);

    usleep(10000);
    return XST_SUCCESS;
}

/*******DATA TRANSMITION*******/
static void UartTransmit_Data(BMI088 *imu){
    printf("BMI088_Roll-> %.2f --- ",-imu->roll);
    printf("BMI088_Pitch-> %.2f\n"  ,-imu->pitch);
}

/****************************PUBLIC FUNCTIONS****************************/
/*
 * Initializes the IMU completly
 */
int BMI088_Init(BMI088 *imu){
    BMI088_Init_I2C(imu);
    BMI088_Init_GYRO(imu);
    BMI088_Init_ACC(imu);
    return XST_SUCCESS;
}

/**
 * @brief Read temperature data from the BMI088 sensor.
 *
 * This public function reads temperature data from the BMI088 sensor.
 *
 * @param imu Pointer to the BMI088 structure.
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */
int BMI088_readTEMP(BMI088 *imu){
    u8 TempData[2];
    int Status;
    int16_t Temp_int16;
    u16 Temp_uint16;

    Status = BMI088_readReg(imu, BMI088_ACC_ADDR, BMI088_TEMP_MSB_REG, TempData, 2);
    if (Status != XST_SUCCESS) {
        printf("Error reading temperature value!\n");
        return XST_FAILURE;
    }

    // 11 bits
    Temp_uint16 = (TempData[0]*8) + (TempData[1]/32);
    if (Temp_uint16>1023)
    {
        Temp_int16 = Temp_uint16 - 2048;
    }
    else
    {
        Temp_int16 = Temp_uint16;
    }
    imu->temperature = (float) Temp_int16 * 0.125f + 23.0f;

    printf("Temperature: %.2f °C\n", imu->temperature);
    return XST_SUCCESS;
}

/**
 * @brief Process data from the BMI088 sensor.
 *
 * This public function reads, fuses the sensors data, filters the data and transmit it to the UART.
 *
 * @param imu Pointer to the BMI088 structure.
 * @return XST_SUCCESS if successful, otherwise XST_FAILURE.
 */
int BMI088_Process(BMI088* imu){
    BMI088_readACC(imu);
    BMI088_readGYRO(imu);

    BMI088_Madgwick_SensorFusion(imu);
    
    BMI088_KalmanFilter(imu);
    
    UartTransmit_Data(imu);
    return XST_SUCCESS;
}


