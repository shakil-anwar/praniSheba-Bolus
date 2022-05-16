/********************************************************************************
Module: MPU6050
Author: 5/4/2015, by KienLTb

Description: MPU6050 Lib for MSP430G2553

********************************************************************************/
#ifndef __MPU6050__H__
#define __MPU6050__H__
/*-----------------------------------------------------------------------------*/
/* Header inclusions */
/*-----------------------------------------------------------------------------*/
#include <msp430.h>
#include "../mspDriver.h"
#include "mpu6050_registers.h"
//#include "types.h"
//typedef  uint8_t    BYTE;
//typedef int16_t     int16_t;
//typedef void*       PVOID;
//typedef void        VOID;
/*-----------------------------------------------------------------------------*/
/* Constant definitions  */
/*-----------------------------------------------------------------------------*/

/*---------- CONFIG VALUE----------*/
/* MPU6050_PWR_MGMT_1 REG */
#define CLKSEL_0 0x00 //Internal 8MHz Osilator
#define CLKSEL_1 0x01 //PLL with X axis gyroscope reference
#define CLKSEL_2 0x02 //PLL with Y axis gyroscope reference
#define CLKSEL_3 0x03 //PLL with Z axis gyroscope reference
#define CLKSEL_4 0x04 //PLL with external 32.768kHz reference
#define CLKSEL_5 0x05 //PLL with external 19.2MHz reference
#define CLKSEL_6 0x06 //Reserved
#define CLKSEL_7 0x07 //Stops the clock and keeps the timing generator in reset

#define TEMP_DIS	 (0x01 << 3) //disables the temperature sensor
#define CYCLE 	 	 (0x01 << 5) //
#define SLEEP 		 (0x01 << 6) //
#define DEVICE_RESET (0x01 << 7) //
#define FIFO_RESET   (0x01 << 2)
#define FIFO_EN      (0x01 << 6)

/*----------MPU6050_CONFIG----------*/
#define EXT_SYNC_SET_INPUT_DISABLE 	(0x00 << 3)// Input disabled
#define EXT_SYNC_SET_TEMP_OUT 		(0x01 << 3)// TEMP_OUT_L[0]
#define EXT_SYNC_SET_GYRO_XOUT 		(0x02 << 3)// GYRO_XOUT_L[0]
#define EXT_SYNC_SET_GYRO_YOUT 		(0x03 << 3)// GYRO_YOUT_L[0]
#define EXT_SYNC_SET_GYRO_ZOUT 		(0x04 << 3)// GYRO_ZOUT_L[0]
#define EXT_SYNC_SET_ACCEL_XOUT 	(0x05 << 3)// ACCEL_XOUT_L[0]
#define EXT_SYNC_SET_ACCEL_YOUT 	(0x06 << 3)// ACCEL_YOUT_L[0]
#define EXT_SYNC_SET_ACCEL_ZOUT 	(0x07 << 3)// ACCEL_ZOUT_L[0]

#define DLPF_CFG_BAND_WIDTH_260HZ 		0x00// BandWidth 260Hz
#define DLPF_CFG_BAND_WIDTH_184HZ  		0x01// BandWidth 184Hz
#define DLPF_CFG_BAND_WIDTH_94HZ 		0x02// BandWidth 94Hz
#define DLPF_CFG_BAND_WIDTH_44HZ 		0x03// BandWidth 44Hz
#define DLPF_CFG_BAND_WIDTH_21HZ 		0x04// BandWidth 21Hz
#define DLPF_CFG_BAND_WIDTH_10HZ 		0x05// BandWidth 10Hz
#define DLPF_CFG_BAND_WIDTH_5HZ 		0x06// BandWidth 5Hz

/*---------MPU6050_GYRO_CONFIG-------*/
#define PS_SEL_SCALE_250	(0x00 << 3)
#define PS_SEL_SCALE_500	(0x01 << 3)
#define PS_SEL_SCALE_1000	(0x02 << 3)
#define PS_SEL_SCALE_2000 	(0x03 << 3)

#define ZG_ST	(0x01 << 5)
#define YG_ST	(0x01 << 6)
#define XG_ST	(0x01 << 7)

/*---------MPU6050_ACCEL_CONFIG------*/
#define AFS_SEL_SCALE_2G	(0x00 << 3)
#define AFS_SEL_SCALE_4G	(0x01 << 3)
#define AFS_SEL_SCALE_8G	(0x02 << 3)
#define AFS_SEL_SCALE_16G	(0x03 << 3)

#define ZA_ST	(0x01 << 5)
#define YA_ST	(0x01 << 6)
#define XA_ST	(0x01 << 7)
/*---------MPU6050_SMPLRT_DIV_CONFIG---*/
#define SET_SAMPLE_RATE_1000HZ	 0x07
#define SET_SAMPLE_RATE_5HZ      199
/*-----------------------------------------------------------------------------*/
/* Macro definitions  */
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/* Global variables  */
/*-----------------------------------------------------------------------------*/
#define GYRO_CONFIG_250      0
#define GYRO_CONFIG_500      1
#define GYRO_CONFIG_1000     2
#define GYRO_CONFIG_2000     3


#define ACC_CONFIG_2G 	    4
#define ACC_CONFIG_4G 	    5
#define ACC_CONFIG_8G	    6
#define ACC_CONFIG_16G	    7


/*-----------------------------------------------------------------------------*/
/* Data type definitions */
/*-----------------------------------------------------------------------------*/
//typedef unsigned char BYTE;
//typedef unsigned short WORD;
//typedef unsigned long DWORD;
//typedef unsigned int UINT;
//typedef BYTE BOOL;
//typedef unsigned char CHAR;
//typedef void VOID;
//
//typedef BYTE* PBYTE;
//typedef WORD* PWORD;
//typedef DWORD* PDWORD;
//typedef UINT* PUINT;
//typedef CHAR* PCHAR;
//typedef VOID* PVOID;
//
//typedef BYTE RESULT;
//
//typedef const BYTE* PCBYTE;
//
//#define FALSE   0
//#define TRUE    1
//#define NULL	0
/* Acc data raw value */
typedef struct ACC_DATA_RAW
{
        int16_t x;
        int16_t y;
        int16_t z;

}ACC_DATA_RAW, *PACC_DATA_RAW;

/* Acc data value in angle */
typedef struct _ACC_DATA_SCALED
{
        float x;
        float y;
        float z;
}ACC_DATA_SCALED, *PACC_DATA_SCALED;

/* Gyro data raw value */
typedef struct GYRO_DATA_RAW
{
        int16_t x;
        int16_t y;
        int16_t z;
}GYRO_DATA_RAW, *PGYRO_DATA_RAW;

/* Gyro data value in angle */
typedef struct _GYRO_DATA_SCALED
{
        float x;
        float y;
        float z;
}GYRO_DATA_SCALED, *PGYRO_DATA_SCALED;

/* Angle data caculate from raw value */
typedef struct _ANGLE
{
        float x;
        float y;
        float z;
}ANGLE, *PANGLE;
/*-----------------------------------------------------------------------------*/
/* Function prototypes  */
/*-----------------------------------------------------------------------------*/

void MPU6050_Init(uint8_t ACC_SCALE_CONFIG, uint8_t GYRO_SCALE_CONFIG);

/* MPU6050 test i2c connection */
uint8_t MPU6050_CheckI2C(void);

/* MPU6050 test configure of register*/
//uint8_t MPU6050_TestRegConfig(void);

/* Get the calibrate value and store to offset*/
//void MPU6050_Calibrate_Gyro(void);
/* Raw Acc Value*/
//void MPU6050_GetAccValueRaw(ACC_DATA_RAW *pValue);

/* Value in degree/s */
//void PMU6050_AccConvertData(ACC_DATA_RAW rawValue, void *scaledData);
//
///* Get rotation of sensor, value in degree*/
//void PMU6050_GetRotationAngle(ACC_DATA_RAW raw, void *pRotationAngle);

/* Raw Gyro Value to m/s^2*/
//void MPU6050_GetGyroValueRaw(GYRO_DATA_RAW *pValue);
void gyroPowerDown(void);
void gyroPowerUP(void);
void testGyro();
//void printAccValue(ACC_DATA_RAW *accData, GYRO_DATA_RAW *gyData);
//int MPU6050_GetFifoCount(void);
//int MPU6050_GetTemp(void);

int MPU6050_fifoCount();
void MPU6050_readFifo();


void gyroGetAccRawValue(uint8_t *buf);
void gyroGetGyroRawValue (uint8_t *buf);

/********New functions for DMP**********/
void MPU6050_initDMP();

/* Convert to m/s^2*/
//void PMU6050_GyroConvertData(GYRO_DATA_RAW rawValue, void *scaledData);
//
///* Complementrary Filter*/
//void Complementary_Filter(ACC_DATA_RAW accData, GYRO_DATA_RAW gyroData, void *pAngle);

#endif // __MPU6050__H__
