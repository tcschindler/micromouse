/*
 * ICM42688.cpp
 *
 *  Created on: Sep 13, 2024
 *      Author: physik
 */

#include "ICM42688.h"
#include "math.h"

//#include <cstdint>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
float _aRes, _gRes;


int8_t ICM42688_WhoAmI()
{
	// set userbank to UB0
	ICM42688_SPIWriteReg(UB0_REG_BANK_SEL, UB0);

	printf("%x\n", ICM42688_SPIReadReg(UB0_REG_WHO_AM_I));

	if(ICM42688_SPIReadReg(UB0_REG_WHO_AM_I) != 0x47)
	{
		printf("Lost Soul\n");
		return -1;
	}
	else
	{
		printf("ICM46288 - ready \n");
		return 0;
	}
}

void ICM42688_Init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
	// select userbank 0
	ICM42688_SetUB(UB0);

	// turn accel and gyro on - low noise mode
	ICM42688_SPIWriteReg(UB0_REG_PWR_MGMT0, 0x0F);

	// accel config
	ICM42688_SPIWriteReg(UB0_REG_ACCEL_CONFIG0, Ascale << 5 | AODR);

	// gyro config
	ICM42688_SPIWriteReg(UB0_REG_GYRO_CONFIG0, Gscale << 5 | GODR);


}

uint8_t ICM42688_SPIReadReg(uint8_t regAdr)
{
	uint8_t regVal;

	// set MSB to READ
	regAdr |= 0x80;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi1, &regAdr, 1, 100);
	HAL_Delay(1);
	HAL_SPI_Receive(&hspi1, &regVal, 1, 100);
	HAL_Delay(1);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	return regVal;
}
int8_t ICM42688_SPIWriteReg(uint8_t regAdr, uint8_t regVal)
{

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi1, &regAdr, 1, 100);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi1, &regVal, 1, 100);
	HAL_Delay(1);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	if(ICM42688_SPIReadReg(regAdr) != regVal)
		return -1;
	else
	return 0;
}

uint8_t ICM42688_CurrentUB()
{
	uint8_t current_ub;

	current_ub = ICM42688_SPIReadReg(UB0_REG_BANK_SEL);
	printf("Current Userbank: %u\n", current_ub);
	return current_ub;
}

uint8_t ICM42688_SetUB(uint8_t desired_userbank)
{
	uint8_t ub;

	ub = ICM42688_SPIWriteReg(UB0_REG_BANK_SEL, desired_userbank);

	return ub;
}

void ICM42688_Selftest(float *accelDiff, float *gyroDiff, float *ratio)
{
	/* MAY NEED TO ADJUST HERE */
	//float aRes = 16.0f/32768.0f, gRes = 2000.0f/32768.0f;

	int16_t data[7] = {0, 0, 0, 0, 0, 0, 0};
	int16_t accelSTest[3] = {0, 0, 0}, gyroSTest[3] = {0, 0, 0};
	int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

	// set aRes and gRes for selftest
	float aResST = 4.0f/32768.0f;
	float gResST = 250.0f/32768.0f;

	ICM42688_SetUB(UB0);																// select userbank 0

	ICM42688_SPIWriteReg(UB0_REG_PWR_MGMT0, 0x0F);										// turn on accel and gyro in low noise mode
	HAL_Delay(1);

	ICM42688_SPIWriteReg(UB0_REG_ACCEL_CONFIG0, AFS_4G << 5 | AODR_1kHz);
	ICM42688_SPIWriteReg(UB0_REG_GYRO_CONFIG0, GFS_250DPS << 5 | GODR_1kHz);

	ICM42688_SPIWriteReg(UB0_REG_GYRO_ACCEL_CONFIG0, 0x44);								// set gyro & accel bandwidth to ODR/10

	ICM42688_readData(data);
	accelNom[0] = data[1];
	accelNom[1] = data[2];
	accelNom[2] = data[3];
	gyroNom[0] = data[4];
	gyroNom[1] = data[5];
	gyroNom[2] = data[6];

	ICM42688_SPIWriteReg(UB0_REG_SELF_TEST_CONFIG, 0x78); 								// run accel self test
	HAL_Delay(100);																		// respond delay
	ICM42688_readData(data);
	accelSTest[0] = data[1];
	accelSTest[1] = data[2];
	accelSTest[2] = data[3];

	ICM42688_SPIWriteReg(UB0_REG_SELF_TEST_CONFIG, 0x07);								// run gyro self test
	HAL_Delay(100);																		// respond delay
	ICM42688_readData(data);
	gyroSTest[0] = data[4];
	gyroSTest[1] = data[5];
	gyroSTest[2] = data[6];

	ICM42688_SPIWriteReg(UB0_REG_SELF_TEST_CONFIG, 0x00);								// end self-test mode and resume to normal mode

	accelDiff[0] = accelSTest[0] - accelNom[0];
	if(accelDiff[0] < 0) accelDiff[0] *= -1;
	accelDiff[1] = accelSTest[1] - accelNom[1];
	if(accelDiff[1] < 0) accelDiff[1] *= -1;
	accelDiff[2] = accelSTest[2] - accelNom[2];
	if(accelDiff[2] < 0) accelDiff[2] *= -1;

	gyroDiff[0] = gyroSTest[0] - gyroNom[0];
	if(gyroDiff[0] < 0) gyroDiff[0] *= -1;
	gyroDiff[1] = gyroSTest[1] - gyroNom[1];
	if(gyroDiff[1] < 0) gyroDiff[1] *= -1;
	gyroDiff[2] = gyroSTest[2] - gyroNom[2];
	if(gyroDiff[2] < 0) gyroDiff[2] *= -1;

	ICM42688_SetUB(UB2);
	data[1] = ICM42688_SPIReadReg(UB2_REG_XA_ST_DATA);
	data[2] = ICM42688_SPIReadReg(UB2_REG_YA_ST_DATA);
	data[3] = ICM42688_SPIReadReg(UB2_REG_ZA_ST_DATA);

	ICM42688_SetUB(UB1);
	data[4] = ICM42688_SPIReadReg(UB1_REG_XG_ST_DATA);
	data[5] = ICM42688_SPIReadReg(UB1_REG_YG_ST_DATA);
	data[6] = ICM42688_SPIReadReg(UB1_REG_ZG_ST_DATA);

	ratio[1] = accelDiff[0] / (1310.0f * powf(1.01f, data[1] - 1) + 0.5f);
	ratio[2] = accelDiff[1] / (1310.0f * powf(1.01f, data[2] - 1) + 0.5f);
	ratio[3] = accelDiff[2] / (1310.0f * powf(1.01f, data[3] - 1) + 0.5f);
	ratio[4] = gyroDiff[0] 	/ (2620.0f * powf(1.01f, data[4] - 1) + 0.5f);
	ratio[5] = gyroDiff[1] /  (2620.0f * powf(1.01f, data[5] - 1) + 0.5f);
	ratio[6] = gyroDiff[2] /  (2620.0f * powf(1.01f, data[6] - 1) + 0.5f);

	for(int i= 0; i < 3; i++)
	{
		accelDiff[i] = accelDiff[i] * aResST *1000.0f;
		gyroDiff[i] = gyroDiff[i] * gResST *1000.0f;
	}

	for(int j=1; j < 7; j++)
	{
		ratio[j] = ratio[j] * 100.0f;
	}

	ICM42688_SetUB(UB0);

	printf("Accel Self-Test:\n");
	printf("Ax diff: %.2f mg\tAy diff: %.2f mg\tAz diff: %.2f mg\n", accelDiff[0], accelDiff[1], accelDiff[2]);
	printf("Should be between 50 and 1200 mg\n");
	printf("Accel ratio:\n");
	printf("Ax ratio: %.0f\tAy ratio: %.0f\tAz ratio: %.0f\n", ratio[1], ratio[2], ratio[3]);
	printf("Should be between 50 and 150%%\n\n");

	printf("Gyro Self-Test:\n");
	printf("Gx diff: %.2f dps\tGy diff: %.2f dps\tGz diff: %.2f dps\n", gyroDiff[0], gyroDiff[1], gyroDiff[2]);
	printf("Should be > 60 dps\n");
	printf("Gyro ratio:\n");
	printf("Gx ratio: %.0f\tGy ratio: %.0f\tGz ratio: %.0f\n", ratio[4], ratio[5], ratio[6]);
	printf("Should be between 50 and 150%%\n\n");
}

float ICM42688_getARes(uint8_t Ascale)
{
	switch (Ascale)
	{
	// Possible accelerometer scales (and their register bit settings) are:
		case AFS_2G:
			 _aRes = 2.0f/32768.0f;
			 return _aRes;
			 break;
		case AFS_4G:
			 _aRes = 4.0f/32768.0f;
			 return _aRes;
			 break;
		case AFS_8G:
			 _aRes = 8.0f/32768.0f;
			 return _aRes;
			 break;
		case AFS_16G:
			 _aRes = 16.0f/32768.0f;
			 return _aRes;
			 break;
		default:
			break;
	}

	return -1.0;
}

float ICM42688_getGRes(uint8_t Gscale)
{
	switch (Gscale)
	{
	// Possible gyro scales (and their register bit settings) are:
		case GFS_15_625DPS:
			_gRes = 15.625f/32768.0f;
			return _gRes;
			break;
		case GFS_31_25DPS:
			_gRes = 31.25f/32768.0f;
			return _gRes;
			break;
		case GFS_62_50DPS:
			_gRes = 62.5f/32768.0f;
			return _gRes;
			break;
		case GFS_125DPS:
			_gRes = 125.0f/32768.0f;
			return _gRes;
			break;
		case GFS_250DPS:
			_gRes = 250.0f/32768.0f;
			return _gRes;
			break;
		case GFS_500DPS:
			_gRes = 500.0f/32768.0f;
			return _gRes;
			break;
		case GFS_1000DPS:
			_gRes = 1000.0f/32768.0f;
			return _gRes;
			break;
		case GFS_2000DPS:
			_gRes = 2000.0f/32768.0f;
			return _gRes;
			break;
		default:
			break;
	}

	return -1.0;
}

void ICM42688_offsetBias(float *aBias, float *gBias)
{
	int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
	int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

	printf("Calculate accel and gyro offset biases: keep sensor flat and motionless!\n");
	HAL_Delay(4000);

	for (int i = 0; i < 128; i++)
	  {
	    ICM42688_readData(temp);
	    sum[1] += temp[1];
	    sum[2] += temp[2];
	    sum[3] += temp[3];
	    sum[4] += temp[4];
	    sum[5] += temp[5];
	    sum[6] += temp[6];
	    HAL_Delay(50);
	  }

	aBias[0] = sum[1]*_aRes/128.0f;
	aBias[1] = sum[2]*_aRes/128.0f;
	aBias[2] = sum[3]*_aRes/128.0f;
	gBias[0] = sum[4]*_gRes/128.0f;
	gBias[1] = sum[5]*_gRes/128.0f;
	gBias[2] = sum[6]*_gRes/128.0f;

	if(aBias[0] > 0.8f)  {aBias[0] -= 1.0f;}  																// Remove gravity from the x-axis accelerometer bias calculation
	if(aBias[0] < -0.8f) {aBias[0] += 1.0f;}  																// Remove gravity from the x-axis accelerometer bias calculation
	if(aBias[1] > 0.8f)  {aBias[1] -= 1.0f;} 																// Remove gravity from the y-axis accelerometer bias calculation
	if(aBias[1] < -0.8f) {aBias[1] += 1.0f;}  																// Remove gravity from the y-axis accelerometer bias calculation
	if(aBias[2] > 0.8f)  {aBias[2] -= 1.0f;}  																// Remove gravity from the z-axis accelerometer bias calculation
	if(aBias[2] < -0.8f) {aBias[2] += 1.0f;}  																// Remove gravity from the z-axis accelerometer bias calculation

	// load offset biases into offset registers (optional, comment out if not desired)
	temp[1] = (int16_t) (-aBias[0] / 0.00048828125f); 														// Ax 0.5 mg resolution
	temp[2] = (int16_t) (-aBias[1] / 0.00048828125f); 														// Ay
	temp[3] = (int16_t) (-aBias[2] / 0.00048828125f); 														// Az
	temp[4] = (int16_t) (-gBias[0] / 0.03125f);       														// Gx 1/32 dps resolution
	temp[5] = (int16_t) (-gBias[1] / 0.03125f);       														// Gy
	temp[6] = (int16_t) (-gBias[2] / 0.03125f);       														// Gz

	ICM42688_SetUB(UB4);

	ICM42688_SPIWriteReg(UB4_REG_OFFSET_USER5, temp[1] & 0x00FF);											// lower Ax byte
	ICM42688_SPIWriteReg(UB4_REG_OFFSET_USER6, temp[2] & 0x00FF);											// lower Ay byte
	ICM42688_SPIWriteReg(UB4_REG_OFFSET_USER8, temp[3] & 0x00FF);											// lower Az byte
	ICM42688_SPIWriteReg(UB4_REG_OFFSET_USER2, temp[5] & 0x00FF);											// lower Gy byte
	ICM42688_SPIWriteReg(UB4_REG_OFFSET_USER3, temp[6] & 0x00FF);											// lower Gz byte
	ICM42688_SPIWriteReg(UB4_REG_OFFSET_USER0, temp[4] & 0x00FF);											// lower Gx byte
	ICM42688_SPIWriteReg(UB4_REG_OFFSET_USER4, (temp[1] & 0x0F00) >> 4 | (temp[6] & 0x0F00) >> 8);			// upper Ax and Gz bytes
	ICM42688_SPIWriteReg(UB4_REG_OFFSET_USER7, (temp[3] & 0x0F00) >> 4 | (temp[2] & 0x0F00) >> 8);			// upper Az and Ay bytes
	ICM42688_SPIWriteReg(UB4_REG_OFFSET_USER1, (temp[5] & 0x0F00) >> 4 | (temp[4] & 0x0F00) >> 8);			// upper Gy and Gx bytes

	ICM42688_SetUB(UB0);

}



void ICM42688_readData(int16_t *destination)
{
	uint8_t rawData[14];  // temperatur, x/y/z accel, x/y/z gyro register data stored here
	uint8_t regAdr = UB0_REG_TEMP_DATA1 | 0x80;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi1, &regAdr, 1, 100);
	HAL_Delay(1);
	HAL_SPI_Receive(&hspi1, rawData, 14, 100);
	HAL_Delay(1);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];  								// Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
	destination[3] = ((int16_t)rawData[6] << 8) | rawData[7];
	destination[4] = ((int16_t)rawData[8] << 8) | rawData[9];
	destination[5] = ((int16_t)rawData[10] << 8) | rawData[11];
	destination[6] = ((int16_t)rawData[12] << 8) | rawData[13];
}
