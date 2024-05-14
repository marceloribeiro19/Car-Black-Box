#ifndef __ASM330LHB_H__
#define __ASM330LHB_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>        // Contains POSIX terminal control definitions (UART)
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

/*
 *  DEFINES
 */
#define I2C_DEV_PATH  "/dev/i2c-1"
#define UART_DEV_PATH "/dev/ttyAMA0"    //dev/ttyS0"
#define ASM330LHB_SLAVE_ADDR            0x6A
#define ASM330LHB_WHO_AM_I_ADDR         0x0F
#define ASM330LHB_WHO_AM_I_CONTENT      0x6B
/*
 * CONFIGURATION
 */
#define CTRL1_XL  			0x10		//Accelerometer control register
#define CTRL2_G 			0x11		//Gyroscope control register

/*
 * OUTPUT REGISTERS
 */
#define ACC_X_REG			0x28
#define ACC_Y_REG			0x2A
#define ACC_Z_REG			0x2C
#define GYRO_X_REG			0x22
#define GYRO_Y_REG			0x24
#define GYRO_Z_REG			0x26
#define TEMP_REG		 	0x20

/*
 * Ranges
 */
#define GYRO_RANGE_125_DPS		10			// +/- 125 deg/s as a special pin
#define GYRO_RANGE_250_DPS		0b00  		// +/- 250 deg/s (default value)
#define GYRO_RANGE_500_DPS		0b01  		// +/- 500 deg/s
#define GYRO_RANGE_1000_DPS	    0b10		// +/- 1000 deg/s
#define GYRO_RANGE_2000_DPS	    0b11 		// +/- 2000 deg/s
#define GYRO_RANGE_4000_DPS	    11			// +/- 4000 deg/s as a special pin
#define ACC_RANGE_2_G	 	  	0b00		// +/- 2g (2.048f)(default value)
#define ACC_RANGE_4_G   		0b10		// +/- 4g (4.096f)
#define ACC_RANGE_8_G  			0b11		// +/- 8g (8.192f)
#define ACC_RANGE_16_G  		0b01	// +/- 16g (16.	384f)

/*
 * ODR (OUTPUT DATA RATE)
 */
#define ACC_ODR_POWER_DOWN		0b0000
#define ACC_ODR_1P6_HZ    		0b1001
#define ACC_ODR_12P5_HZ   		0b0001
#define ACC_ODR_26_HZ    		0b0010
#define ACC_ODR_52_HZ     		0b0011
#define ACC_ODR_104_HZ   		0b0100
#define ACC_ODR_208_HZ    		0b0101
#define ACC_ODR_416_HZ    		0b0110
#define ACC_ODR_833_HZ    		0b0111
#define ACC_ODR_1670_HZ   		0b1000
#define GYRO_ODR_POWER_DOWN     0b0000
#define GYRO_ODR_12P5_HZ 		0b0001
#define GYRO_ODR_26_HZ  		0b0010
#define GYRO_ODR_52_HZ  		0b0011
#define GYRO_ODR_104_HZ 		0b0100
#define GYRO_ODR_208_HZ  		0b0101
#define GYRO_ODR_416_HZ  		0b0110
#define GYRO_ODR_833_HZ  		0b0111
#define GYRO_ODR_1670_HZ 		0b1000

/* CONSTANTS */
#define G_ACCELERATION		9.81f
#define SAMPLE_TIME_S		0.01	//1
#define RAD_TO_DEG			57.2957795131
#define DEG_TO_RAD			0.0174532925f
#define alpha 				0.2f
#define LSB 				32768	//2^15

typedef struct imuStruct
{
    /* kernel driver file descriptors */
    int i2c_fd;
    int uart_fd;
    
    __u16 gyroRange;
    float accRange; 
   
    float temp_celsius;
    float gyro[3];
    float acc[3];

    /* COMPLEMENTARY FILTER */
    float thetaHat_rad;
    float phiHat_rad;
    
    /* KALMAN FILTER */
    float Xant_Roll, Xant_Pitch, Xant_Yaw;
	float Pant_Roll, Pant_Pitch, Pant_Yaw;

    char RollPitchYaw [66];
}ASM330LHB;

/* Function Prototypes */
__u8 ASM330LHB_Init(ASM330LHB *imu, __u8 GYRO_ODR, __u8 ACC_ODR, __u8 GYRO_RANGE, __u8 ACC_RANGE);
__u8 ASM330LHB_Process(ASM330LHB *imu);
__u8 ASM330LHB_error(char *message, int file_descriptor) ;
#endif /* __ASM330LHB_H__ */
