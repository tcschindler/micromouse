/*
 * ICM42688.h
 *
 *  Created on: Sep 13, 2024
 *      Author: physik
 */

#ifndef SRC_ICM42688_H_
#define SRC_ICM42688_H_

#include "main.h"
#include "ICM42688_Registers.h"
//#include <cstdint>

/* USERBANK */
#define UB0 0x00
#define UB1 0x01
#define UB2 0x02
#define UB3	0x03
#define UB4	0x04

/* ACCEL SCALE */
#define AFS_2G  		0x03
#define AFS_4G  		0x02
#define AFS_8G  		0x01
#define AFS_16G 		0x00 	/* DEFAULT */

/* GYRO SCALE */
#define GFS_2000DPS   	0x00   	/* DEFAULT */
#define GFS_1000DPS   	0x01
#define GFS_500DPS    	0x02
#define GFS_250DPS    	0x03
#define GFS_125DPS    	0x04
#define GFS_62_50DPS  	0x05
#define GFS_31_25DPS  	0x06
#define GFS_15_625DPS 	0x07

/* ACCEL ODR */
// Low Noise mode
#define AODR_32kHz    	0x01
#define AODR_16kHz    	0x02
#define AODR_8kHz     	0x03
#define AODR_4kHz     	0x04
#define AODR_2kHz     	0x05
#define AODR_1kHz     	0x06  	/* DEFAULT */
//Low Noise or Low Power modes
#define AODR_500Hz    	0x0F
#define AODR_200Hz    	0x07
#define AODR_100Hz    	0x08
#define AODR_50Hz     	0x09
#define AODR_25Hz     	0x0A
#define AODR_12_5Hz   	0x0B
// Low Power mode
#define AODR_6_25Hz   	0x0C
#define AODR_3_125Hz  	0x0D
#define AODR_1_5625Hz 	0x0E

/* GYRO ODR */
#define GODR_32kHz  	0x01
#define GODR_16kHz  	0x02
#define GODR_8kHz   	0x03
#define GODR_4kHz   	0x04
#define GODR_2kHz   	0x05
#define GODR_1kHz   	0x06 	/* DEFAULT */
#define GODR_500Hz  	0x0F
#define GODR_200Hz  	0x07
#define GODR_100Hz  	0x08
#define GODR_50Hz   	0x09
#define GODR_25Hz   	0x0A
#define GODR_12_5Hz 	0x0B




int8_t ICM42688_WhoAmI(void);

void ICM42688_Init(uint8_t Ascale, uint8_t GScale, uint8_t AODR, uint8_t GODR);
float ICM42688_getARes(uint8_t Ascale);
float ICM42688_getGRes(uint8_t Gscale);

void ICM42688_offsetBias(float *aBias, float *gBias);

// USERBANK FUNCTIONS
uint8_t ICM42688_CurrentUB();
uint8_t ICM42688_SetUB(uint8_t desired_userbank);

// SPI FUNCTIONS
uint8_t ICM42688_SPIReadReg(uint8_t regAdr);
int8_t ICM42688_SPIWriteReg(uint8_t regAdr, uint8_t regVal);

void ICM42688_readData(int16_t *destination);

void ICM42688_Selftest(float *accelDiff, float *gyroDiff, float *ratio);



#endif /* SRC_ICM42688_H_ */
