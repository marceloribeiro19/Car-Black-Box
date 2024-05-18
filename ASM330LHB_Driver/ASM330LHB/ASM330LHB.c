#include "ASM330LHB.h"
#include "../kalman/kalman.h"
#include "../Madgwick_Filter/madgwick.h"

__u8 ASM330LHB_error(char *message, int file_descriptor);

/*                     *
 *  PRIVATE FUNCTIONS  * 
 *                     */    

/**
 * @brief Reads one or more registers of an I2C slave device and verifies the Read operation.
 * @param file I2C file descriptor.
 * @param reg Register address to read.
 * @param buf Buffer containing data read.
 * @param N Number of bytes to read.
 * @return Returns 0 if the read operation was successful, otherwise returns -1.
 */
static __u8 ASM330LHB_i2cRead(ASM330LHB *imu, __u8 reg, __u8 *buf, __u8 N){ 
    /* write to register*/  
    if (write(imu->i2c_fd, &reg, 1) != 1)
        return ASM330LHB_error("[ASM330LHB_i2cRead] - Error writing I2C Bus.",imu->i2c_fd);
    /* read the value back*/    
    if ((read(imu->i2c_fd, buf, N) != N))
        return ASM330LHB_error("[ASM330LHB_i2cRead] - Error reading I2C Bus.",imu->i2c_fd);
    return 0;
}

/**
 * @brief Writes to one or more registers of an I2C slave device and verifies the write operation.
 * @param file I2C file descriptor.
 * @param reg Register address to write to.
 * @param data Buffer containing data to write.
 * @return Returns 0 if the write operation was successful, otherwise returns -1.
 */
static inline __u8 ASM330LHB_i2cWrite(ASM330LHB *imu, __u8 reg, __u8 data) {   
    __u8 buffer[2];
    buffer[0] = reg;
    buffer[1] = data;
    if(write(imu->i2c_fd, buffer, 2) != 2)
        return ASM330LHB_error("[ASM330LHB_i2cWrite] - Error writing I2C Bus.",imu->i2c_fd);
    return 0;
}

/**
 * @brief  Initializes the I2C communication
 * @param  imu Pointer to the ASM330LHB sensor structure.
 * @return Returns 0 if the communication was stablished, otherwise returns -1.
 */
static __u8 ASM330LHB_i2cInit(ASM330LHB *imu){
    /* Load the i2c-dev kernel module*/
    system("modprobe i2c-dev");

    /* Open i2c bus */
    imu->i2c_fd = open(I2C_DEV_PATH, O_RDWR);                          
    if (imu->i2c_fd < 0)
        return ASM330LHB_error("Error opening I2C Bus.", imu->i2c_fd); 
 
    /* Select the I2C Slave device */
    if (ioctl(imu->i2c_fd, I2C_SLAVE, ASM330LHB_SLAVE_ADDR) < 0) 
        return ASM330LHB_error("Error setting slave address.", imu->i2c_fd); 

    /* Verify WHO_AM_I register */
    __u8 reg = ASM330LHB_WHO_AM_I_ADDR;
    char buf[1];
    if (write(imu->i2c_fd, &reg, 1) != 1)
        return ASM330LHB_error("Error Writing to WHO_AM_I.", imu->i2c_fd);       

    /* Read the register and compare with the expected value */
    if ((read(imu->i2c_fd, buf, 1) != 1) && buf[0] != ASM330LHB_WHO_AM_I_CONTENT)
        return ASM330LHB_error("Error reading WHO_AM_I.", imu->i2c_fd);

    printf("ASM330LHB CONNECTED! WHO_AM_I : 0x%x\n",buf[0]);
    return 0;
}

/**
 * @brief  Reads data from the ASM330LHB sensor registers and converts it to meaningful values.
 * @param  imu Pointer to the ASM330LHB sensor structure.
 * @return Returns 0 upon successful data reading and conversion, -1 otherwise.
 */
static __u8 ASM330LHB_readData(ASM330LHB *imu){
    __u8 regData[14];
    __u8 Status = ASM330LHB_i2cRead(imu, TEMP_REG, regData, 14);
    if(Status != 0)
        return ASM330LHB_error("Error Reading I2C",imu->i2c_fd);
    
    /*
    * The value is expressed as a 16-bit word in two’s complement so a signed array in needed
    */
    int16_t tempRaw = ((int16_t)regData[1] << 8 | (int16_t)regData[0]);
    int16_t gyroRaw[3];
    gyroRaw[0] =((int16_t) regData[3] << 8 | (int16_t) regData[2]); /*X-AXIS*/
    gyroRaw[1] =((int16_t) regData[5] << 8 | (int16_t) regData[4]); /*Y-AXIS*/
    gyroRaw[2] =((int16_t) regData[7] << 8 | (int16_t) regData[6]); /*Z-AXIS*/
    int16_t accRaw[3];
    accRaw[0] =((int16_t) regData[9] << 8  | (int16_t) regData[8]); /*X-AXIS*/
    accRaw[1] =((int16_t) regData[11] << 8 | (int16_t) regData[10]); /*Y-AXIS*/
    accRaw[2] =((int16_t) regData[13] << 8 | (int16_t) regData[12]); /*Z-AXIS*/

    /*
     * CONVERT RAW TO ºC
     * 1st We take of the offset from the raw data - 0 LSB -> 25º
     * 2nd We multiply by the sensitivity which is 1/ (LSB/ºC) (page 12 Datasheet)
     */
    imu->temp_celsius = 0.00390625f *( (float) tempRaw - 0) + 25.0f;
    
    /*
     * RAW TO dps(º / s) CONVERTION
     * Configuration is at 500dps and we have 16-1 bits of data(since we lose 1 bit to the signal).
     * Sensitivity calculus(dps / LSB)
     * Sense = (500/ 2¹5) = 0,015... -> 1/0,015 = 65.53(65.5360000107)
     */
    imu->gyro_dps[0] = gyroRaw[0] * ((float)imu->gyroRange / LSB);
    imu->gyro_dps[1] = gyroRaw[1] * ((float)imu->gyroRange / LSB);
    imu->gyro_dps[2] = gyroRaw[2] * ((float)imu->gyroRange / LSB);
   // printf("[%f]\n",imu->gyro_dps[0]);
    /**
     * RAW TO m/s² CONVERTION
     * Configuration is at +/-4.096 g and we have 16-1 bits of data(since we lose 1 bit to the signal).
     * Sensitivity calculus(m/s² / LSB) is the m/s² we have per unit
     * Sense = (4.096*9.81 / 2¹5)
     */
    imu->acc_ms2[0] = accRaw[0] * (imu->accRange / LSB) * G_ACCELERATION;
    imu->acc_ms2[1] = accRaw[1] * (imu->accRange / LSB) * G_ACCELERATION;
    imu->acc_ms2[2] = accRaw[2] * (imu->accRange / LSB) * G_ACCELERATION;
   // printf("[%f]\n",imu->acc_ms2[0]);
	return Status;
}

/*                    *
 *  PUBLIC FUNCTIONS  * 
 *                    */       

/**
 * @brief Error Handler.
 * @param message Error message.
 * @param file_descriptor To close driver F.D in case of existing one in the current error.
 * @return Returns -1.
*/
__u8 ASM330LHB_error(char *message, int file_descriptor){
    fprintf( stderr, "[ERROR]: %s\n", message );
    if (file_descriptor >= 0) //Checks if file descriptor is valid before closing it
        close(file_descriptor);
    exit(EXIT_FAILURE);
}

/**
 * @brief Initializes the imu communication and stores and writes to the device the user's predefined configuration
 * @param imu Pointer to the ASM330LHB sensor structure.
 * @param GYRO_ODR    Gyroscope Output Data Rate
 * @param ACC_ODR     Accelerometer Output Data Rate
 * @param GYRO_RANGE  Gyroscope Range (125dps -> 4000dps)
 * @param ACC_RANGE   Accelerometer Range(+/- 2g -> +/- 16g)
 * @return Returns 0 upon successful initialization, -1 otherwise.
 */
__u8 ASM330LHB_Init(ASM330LHB *imu, __u8 GYRO_ODR, __u8 ACC_ODR, __u8 GYRO_RANGE, __u8 ACC_RANGE){
    if(ASM330LHB_i2cInit(imu) != 0)
        return ASM330LHB_error("Error Initializing I2C", imu->i2c_fd);

    /* ACCELEROMETER CONFIGURATION */
    switch (ACC_RANGE){
        case ACC_RANGE_2_G:  	imu->accRange = 2.048f;  break;
        case ACC_RANGE_4_G:  	imu->accRange = 4.096f;  break;
        case ACC_RANGE_8_G:  	imu->accRange = 8.192f;  break;
        case ACC_RANGE_16_G: 	imu->accRange = 16.384f; break;
    }
    __u8 data = (ACC_ODR<<4 | ACC_RANGE<<2);
    if(ASM330LHB_i2cWrite(imu, CTRL1_XL, data) != 0)
        return ASM330LHB_error("Error in IMU Accelerometer configuration.", imu->i2c_fd);

    /* GYROSCOPE CONFIGURATION */
    switch (GYRO_RANGE){
        case GYRO_RANGE_125_DPS:	imu->gyroRange = 125;	    data = (GYRO_ODR <<4 | 0b00<<2 	     | 0b1 << 1 | 0);	//activates 125 dps special pin
        break;
        case GYRO_RANGE_250_DPS:	imu->gyroRange = 250;	    data = (GYRO_ODR <<4 | GYRO_RANGE<<2 | 0b00 << 2);
        break;
        case GYRO_RANGE_500_DPS:	imu->gyroRange = 500;	    data = (GYRO_ODR <<4 | GYRO_RANGE<<2 | 0b00 << 2);
        break;
        case GYRO_RANGE_1000_DPS:	imu->gyroRange = 1000;	    data = (GYRO_ODR <<4 | GYRO_RANGE<<2 | 0b00 << 2);
        break;
        case GYRO_RANGE_2000_DPS: 	imu->gyroRange = 2000; 	    data = (GYRO_ODR <<4 | GYRO_RANGE<<2 | 0b00 << 2);
        break;
        case GYRO_RANGE_4000_DPS:	imu->gyroRange = 4000;	    data = (GYRO_ODR <<4 | 0b00<<2 	     | 0b0 << 1 | 1);	//activates 4000 dps special pin
        break;
    }
    if(ASM330LHB_i2cWrite(imu, CTRL2_G, data) != 0)
        return ASM330LHB_error("Error in IMU Gyroscope configuration.", imu->i2c_fd);
    
    /*if(ASM330LHB_readData(imu) != 0)
        return ASM330LHB_error("Error in the First data reading.", imu->i2c_fd);
    if(ASM330LHB_calibrate(imu) != 0)
        return ASM330LHB_error("Error Calibrating IMU.", imu->i2c_fd);
*/
    return 0;
}

/**
 * @brief Processes the IMU data and updates the orientation quaternion.
 * 
 * This function reads data from the ASM330LHB sensor and updates the orientation quaternion using the Madgwick filter algorithm.
 * If there is an error in reading the data, it returns an error code and logs the error message.
 * 
 * @param imu Pointer to the ASM330LHB sensor structure containing sensor data and configuration.
 * @return Returns 0 upon successful processing, or an error code if data reading fails and exit() is called.
 */
__u8 ASM330LHB_Process(ASM330LHB *imu){
    if (ASM330LHB_readData(imu) != 0)
        return ASM330LHB_error("Error in data reading.", imu->i2c_fd);

    MadgwickQuaternionUpdate(imu);
    return 0;
}
