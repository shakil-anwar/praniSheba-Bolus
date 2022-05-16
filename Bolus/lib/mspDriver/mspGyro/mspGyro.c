/********************************************************************************

Product: MPU6050
Module:
Created: 4/6/2015, by KienLTb

Description: Driver MPU6050 for MSP430

********************************************************************************/
/*-----------------------------------------------------------------------------*/
/* Header inclusions */
/*-----------------------------------------------------------------------------*/
//#include "msp430g2553.h"
#include "mspGyro.h"
#include "math.h" // For atan();
/*-----------------------------------------------------------------------------*/
/* Local Constant definitions */
/*-----------------------------------------------------------------------------*/
/* AFS_SEL | Full Scale Range | LSB Sensitivity
* --------+------------------+----------------
* 0       | +/- 2g           | 16384 LSB/mg
* 1       | +/- 4g           | 8192 LSB/mg
* 2       | +/- 8g           | 4096 LSB/mg
* 3       | +/- 16g          | 2043 LSB/mg
*/
#define SCALED_ACC_2G       16384
#define SCALED_ACC_4G       8192
#define SCALED_ACC_8G       4096
#define SCALED_ACC_16G      2043

/* FS_SEL | Full Scale Range   | LSB Sensitivity
* -------+--------------------+----------------
* 0      | +/- 250 degrees/s  | 131 LSB/deg/s
* 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
* 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
* 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
*/
#define SCALED_GYRO_250         131.0
#define SCALED_GYRO_500         65.5
#define SCALED_GYRO_1000        32.8
#define SCALED_GYRO_2000        16.4

/*
*   Note:
*          |   ACCELEROMETER    |           GYROSCOPE
* DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
* ---------+-----------+--------+-----------+--------+-------------
* 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
* 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
* 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
* 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
* 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
* 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
* 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
* 7        |   -- Reserved --   |   -- Reserved --   | Reserved
*/


#define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
#define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
#define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]

/*-----------------------------------------------------------------------------*/
/* Local Macro definitions */
/*-----------------------------------------------------------------------------*/
// Note: This Macro use for Bigendian but MSP430 is little endien => revert
#define CONVERT_TO_16BIT(MSB, LSB)              (((int16_t)(MSB) << 8) | (int16_t)(LSB))
#define ABS(x) (x < 0 ? -x : x)
/*-----------------------------------------------------------------------------*/
/* Local Data type definitions */
/*-----------------------------------------------------------------------------*/
#define dt 0.015f
/*-----------------------------------------------------------------------------*/
/* Global variables */
/*-----------------------------------------------------------------------------*/

/* Offset value to calibrate Gyro */
static int16_t Gyro_OffsetValueX = 0;
static int16_t Gyro_OffsetValueY = 0;
static int16_t Gyro_OffsetValueZ = 0;

/* Off set value to calibrate Acc*/
static int16_t Acc_OffsetValueX = 0;
static int16_t Acc_OffsetValueY = 0;
static int16_t Acc_OffsetValueZ = 0;

/* Scale Value config for ACC - default is 2G*/
static float Acc_scaleValue  = SCALED_ACC_4G;

/* Scale Value config for GYRO - default is 2G*/
static float Gyro_scaleValue = SCALED_GYRO_2000;

ACC_DATA_RAW accData;
GYRO_DATA_RAW gyroData;
/*-----------------------------------------------------------------------------*/
/* Function prototypes */
/*-----------------------------------------------------------------------------*/
int I2C_ReadData(uint8_t *buf, int address, uint8_t reg, int len);
int I2C_WriteByte(uint8_t value, uint8_t address, uint8_t reg);
void MPU6050_Init(uint8_t ACC_SCALE_CONFIG, uint8_t GYRO_SCALE_CONFIG);

uint8_t MPU6050_CheckI2C(void);

/* MPU6050 test configure of register*/
uint8_t MPU6050_TestRegConfig(void);

void MPU6050_Calibrate_Gyro(void);
/* Raw Acc Value*/
void MPU6050_GetAccValueRaw(ACC_DATA_RAW *pValue);

/* Value in degree/s */
void PMU6050_AccConvertData(ACC_DATA_RAW *rawValue, PACC_DATA_SCALED scaledData);

/* Raw Gyro Value to m/s^2*/
void MPU6050_GetGyroValueRaw(GYRO_DATA_RAW *pValue);

/* Convert to m/s^2*/
void PMU6050_GyroConvertData(GYRO_DATA_RAW rawValue);

// read I2C error code
void readErrorCode(int eCode);


uint8_t gyroReadBuf[10];

/*-----------------------------------------------------------------------------*/
/* Function implementations                                                                                                */
/*-----------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------
Function        : MPU6050_Init
Purpose         : Init and configure for MPU6050
Parameters      : None
Return          : Note
--------------------------------------------------------------------------------*/

int I2C_ReadData(uint8_t *buf, int address, uint8_t reg, int len)
{
    int err = 0;
//    char buf[2] = {reg};
    *buf = reg;
    err = i2cReadGyroData(address,buf,len);
    readErrorCode(err);
    return 0;
}

int I2C_WriteByte(uint8_t value, uint8_t address, uint8_t reg)
{
//    I2C_ReadData(gyroReadBuf, MPU6050_ADDRESS, 107,1);
//    SerialPrint("PWR_MGMT_1 value: ");
//    SerialPrintlnU8(gyroReadBuf[0]);
    char buf[3];
    int err;
    buf[0] = (char)reg;
    buf[1] = (char)value;
    err = i2cWriteGyroData(address,buf,2);
    readErrorCode(err);
    return 0;
}

int I2C_ReWriteByte(uint8_t reg, uint8_t bit, bool value)
{
    char buf;
    I2C_ReadData(&buf, MPU6050_ADDRESS, reg,1);
    buf &= ~(0x01<<bit);
    buf |= ((value & 0x01)<<bit);
    I2C_WriteByte(buf,MPU6050_ADDRESS,reg);
}

void testGyro()
{
//    uint8_t byBuff;

//    I2C_WriteByte(DEVICE_RESET, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1);
//    delay(110);
//    I2C_WriteByte(0x07, MPU6050_ADDRESS, MPU6050_SIGNAL_PATH_RESET);
//    delay(110);

//    ACC_DATA_SCALED accScaleData;

    MPU6050_GetAccValueRaw(&accData);
    MPU6050_GetGyroValueRaw(&gyroData);
//    PMU6050_AccConvertData(&accData, &accScaleData);
//    PMU6050_GyroConvertData(gyroData);

    printAccValue(&accData, &gyroData);

//    MPU6050_Init(AFS_SEL_SCALE_4G, GYRO_CONFIG_1000);

}


void MPU6050_Init(uint8_t ACC_SCALE_CONFIG, uint8_t GYRO_SCALE_CONFIG)
{
    uint8_t readBuff;
//    I2C_WriteByte(0x07, MPU6050_ADDRESS, MPU6050_SIGNAL_PATH_RESET);

    // Reset MPU6050;
    I2C_WriteByte(DEVICE_RESET, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1);
    delay(500);

    //Set sample rate
//    I2C_WriteByte(SET_SAMPLE_RATE_1000HZ, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV);

    //Config Low pass filter
//    I2C_WriteByte(EXT_SYNC_SET_INPUT_DISABLE + DLPF_CFG_BAND_WIDTH_10HZ, MPU6050_ADDRESS, MPU6050_CONFIG);

//    I2C_ReadData(&readBuff, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,1);
//    readBuff &= 0xF8;
    I2C_WriteByte(CLKSEL_1, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1);
    delay(1000);

    //Set sample rate
    I2C_WriteByte(SET_SAMPLE_RATE_5HZ, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV);

    //Config Low pass filter
    I2C_WriteByte(EXT_SYNC_SET_GYRO_XOUT + DLPF_CFG_BAND_WIDTH_44HZ, MPU6050_ADDRESS, MPU6050_CONFIG);

    //Config Accel
    I2C_WriteByte(ACC_SCALE_CONFIG, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG);

    //Congfig Gyro
    I2C_WriteByte(GYRO_SCALE_CONFIG, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG);

//    I2C_WriteByte(0x78,MPU6050_ADDRESS,MPU6050_FIFO_EN);

    //INT CONFIG ENABLE
//    I2C_WriteByte(0x00, MPU6050_ADDRESS,MPU6050_INT_PIN_CFG);

    //Enable Interrupt
//    I2C_WriteByte(0x01, MPU6050_ADDRESS,MPU6050_INT_ENABLE);

    // Enable MPU6050;
//    I2C_ReadData(&readBuff, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,1);
//    readBuff &= 0b10111111;
//    I2C_WriteByte(readBuff, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1);
    // Clearing FiFo
//    I2C_WriteByte(FIFO_RESET|FIFO_EN,MPU6050_ADDRESS,MPU6050_USER_CTRL);
//    do
//    {
//        I2C_ReadData(gyroReadBuf,MPU6050_ADDRESS,MPU6050_USER_CTRL,1);
//        delay(100);
//    }
//    while(gyroReadBuf[0] & FIFO_RESET);


    delay(1000);
//    I2C_WriteByte(0x78,MPU6050_ADDRESS,MPU6050_FIFO_EN);
//    I2C_ReadData(&readBuff, MPU6050_ADDRESS, MPU6050_FIFO_EN,1);

//    I2C_WriteByte(FIFO_EN,MPU6050_ADDRESS,MPU6050_USER_CTRL);
//    SerialPrint("FiFo EN: "); SerialPrintlnU8(readBuff);

//    I2C_WriteByte(CLKSEL_0, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1);
//    delay(2000);


//    gyroPowerUP();
//      delay(110);
//      MPU6050_Calibrate_Gyro();
//      delay(1000);
//      I2C_ReadData(gyroReadBuf, MPU6050_ADDRESS, 107,1);
//      SerialPrint("PWR_MGMT_1 value: ");
//      SerialPrintlnU8(gyroReadBuf[0]);
//
//      I2C_ReadData(&readBuff, MPU6050_ADDRESS, MPU6050_FIFO_EN,1);
//      SerialPrint("FiFo EN: "); SerialPrintlnU8(readBuff);
//
//      I2C_ReadData(&readBuff, MPU6050_ADDRESS, MPU6050_USER_CTRL,1);
//      SerialPrint("USER_CTRL: "); SerialPrintlnU8(readBuff);
//      MPU6050_Calibrate_Gyro();
}

int MPU6050_fifoCount()
{
    static uint32_t timeout = 0;
    uint16_t count;
    uint8_t *buf =(uint8_t *)count;
    if((millis()-timeout)>2000)
    {
        I2C_ReadData(buf,MPU6050_ADDRESS,MPU6050_FIFO_COUNTH,2);
        SerialPrint("FiFo Count: ");
        SerialPrintlnU16(count);
        timeout = millis();
        return count;
    }
    return 0;
}


void MPU6050_readFifo(int fifoCount)
{
    int i = 0;
    uint8_t *buff = (uint8_t *)&accData;
    uint8_t *buff1 = (uint8_t *)&gyroData;
    ACC_DATA_SCALED accScaleData;
    while(i < fifoCount)
    {
        I2C_ReadData(buff,MPU6050_ADDRESS,MPU6050_FIFO_R_W,6);
        I2C_ReadData(buff1,MPU6050_ADDRESS,MPU6050_FIFO_R_W,6);
        PMU6050_AccConvertData(&accData, &accScaleData);
        PMU6050_GyroConvertData(gyroData);
        i++;
    }
}

void MPU6050_initDMP()
{
    uint8_t buff;
    // Reset Device
    I2C_ReWriteByte(MPU6050_PWR_MGMT_1,7,true);
    delay(30);
    I2C_ReWriteByte(MPU6050_PWR_MGMT_1,6,false);


    // get MPU hardware revision
    buff = 0;
    buff |= 0x10 | 0x20 | 0x40;              // bank, userbank, prefetch
    I2C_WriteByte(buff, MPU6050_ADDRESS,MPU6050_BANK_SEL);
    // set memory start address
    I2C_WriteByte(0x06,MPU6050_ADDRESS,MPU6050_MEM_START_ADDR);
    // resetting memory bank
    buff = 0;
    I2C_WriteByte(buff, MPU6050_ADDRESS,MPU6050_BANK_SEL);

    // setting slave address
    I2C_WriteByte(0x7F,MPU6050_ADDRESS, MPU6050_I2C_SLV0_ADDR);
    //disabliing I2C master mode
    I2C_ReWriteByte(MPU6050_USER_CTRL, 5, false);
    // re-set slave address
    I2C_WriteByte(0x68,MPU6050_ADDRESS, MPU6050_I2C_SLV0_ADDR);
    // re-set I2C master
    I2C_ReWriteByte(MPU6050_USER_CTRL, 1, true);
    delay(20);

    //Setting clock source to Z Gyro
    I2C_ReadData(&buff, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,1);
    buff &= 0xF8;
    I2C_WriteByte(buff | CLKSEL_3, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1);

    //Setting DMP and FIFO_OFLOW interrupts enabled

    //Setting sample rate to 200Hz
    I2C_WriteByte(4, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV);
    //Setting external frame sync to TEMP_OUT_L[0] and  DLPF bandwidth to 42Hz
    I2C_WriteByte(((0x1 << 3) | 0x03) ,MPU6050_ADDRESS, MPU6050_CONFIG);
    //Setting gyro sensitivity to +/- 2000 deg/sec
    I2C_WriteByte(PS_SEL_SCALE_2000, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG);
    // load DMP code into memory banks

}


/*--------------------------------------------------------------------------------
Function    : MPU6050_CheckI2C
Purpose     : Check I2C communication
Parameters  : None
Return      : Value of WHO_AM_I registor (0x68)
-------------------------------------------------------------------------------*/
uint8_t MPU6050_CheckI2C(void)
{
        uint8_t byBuff;
        int errorCode;
        errorCode = I2C_ReadData(&byBuff, MPU6050_ADDRESS, MPU6050_WHO_AM_I,1);
        if(byBuff == 0x68)
        {
            SerialPrintln("MPU6050 is connected");
            return 0;
        }
        else
        {
            SerialPrintln("MPU6050 is not connected");
        }
        return (uint8_t) errorCode;
}


/*--------------------------------------------------------------------------------
Function        : MPU6050_TestRegConfig
Purpose     : Check the config of some register:
- MPU6050_RA_SMPLRT_DIV == 0x01:
-  MPU6050_RA_CONFIG == 0x03;
- MPU6050_RA_GYRO_CONFIG == 0x01;
- MPU6050_RA_ACCEL_CONFIG == 0x00;
Parameters  : None
Return      : 0 if pass and 1 if failt
-------------------------------------------------------------------------------*/

uint8_t MPU6050_TestRegConfig(void)
{
                uint8_t byBuff, ret;
                I2C_ReadData(&byBuff, MPU6050_ADDRESS, MPU6050_SMPLRT_DIV,1);
                ret =  (byBuff == 0x01 ? 0: 1);
        __delay_cycles(10000);
                
                I2C_ReadData(&byBuff, MPU6050_ADDRESS, MPU6050_CONFIG,1);
                ret =  (byBuff == 0x03 ? 0: 1);
        __delay_cycles(10000);
                
                
                I2C_ReadData(&byBuff, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG,1);
                ret =  (byBuff == 0x01 ? 0: 1);
        __delay_cycles(10000);
                
                I2C_ReadData(&byBuff, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG,1);
                ret =  (byBuff == 0x00 ? 0: 1);
        __delay_cycles(10000);
                
                I2C_ReadData(&byBuff, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,1);
                ret =  (byBuff == 0x00 ? 0: 1);
        __delay_cycles(10000);
                
                return ret;
}

/*--------------------------------------------------------------------------------
Function    : MPU6050_GetAccValueRaw
Purpose     : Get raw value x, y, z of accel
Parameters  : PACC_DATA_RAW - pointer to a struct store acc raw data
Return      : NULL
--------------------------------------------------------------------------------*/
void MPU6050_GetAccValueRaw(ACC_DATA_RAW *pValue)
{
        struct ACC_DATA_RAW *accPtr;
        accPtr =   pValue;
        uint8_t pBuff[6];
        I2C_ReadData(&pBuff[0], MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 6);
//        I2C_ReadData(&pBuff[2], MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_H, 2);
//        I2C_ReadData(&pBuff[4], MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_H, 2);

        accPtr->x = (int16_t)(CONVERT_TO_16BIT(pBuff[0], pBuff[1]) - Acc_OffsetValueX);
        accPtr->y = (int16_t)(CONVERT_TO_16BIT(pBuff[2], pBuff[3]) - Acc_OffsetValueY);
        accPtr->z = (int16_t)(CONVERT_TO_16BIT(pBuff[4], pBuff[5]) - Acc_OffsetValueZ);
}



void gyroGetAccRawValue(uint8_t *buf)
{
    I2C_ReadData(buf, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 6);
}

void gyroGetGyroRawValue (uint8_t *buf)
{
    I2C_ReadData(buf, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 6);
}


int MPU6050_GetFifoCount(void)
{
    uint8_t buff[2];
    I2C_ReadData(buff, MPU6050_ADDRESS, MPU6050_FIFO_COUNTH, 2);
    return  ((int16_t)CONVERT_TO_16BIT(buff[0], buff[1]));
}


int MPU6050_GetTemp(void)
{
    uint8_t buff[2];
    I2C_ReadData(buff, MPU6050_ADDRESS, MPU6050_TEMP_OUT_H, 2);
    int16_t temp;
    temp =  (int16_t)CONVERT_TO_16BIT(buff[0], buff[1]);
    SerialPrint("Temp: ");
    SerialPrintlnS16(temp);
    return temp;
}

/*--------------------------------------------------------------------------------
Function    : PMU6050_AccConvertData
Purpose     : Scaled data to radian value
Parameters  : ACC_DATA_RAW , void *scaledData
Return      : NULL
--------------------------------------------------------------------------------*/

void PMU6050_AccConvertData(ACC_DATA_RAW *rawValue, PACC_DATA_SCALED scaledData)
{
        scaledData->x = (float)rawValue->x / Acc_scaleValue;
        scaledData->y = (float)rawValue->y / Acc_scaleValue;
        scaledData->z = (float)rawValue->z / Acc_scaleValue;
        SerialPrint( " ax: "); SerialPrintFloat(scaledData->x, 2);
        SerialPrint( " ay: "); SerialPrintFloat(scaledData->y, 2);
        SerialPrint( " az: "); SerialPrintFloat(scaledData->z, 2);
}
/*--------------------------------------------------------------------------------
Function    : PMU6050_GetAccValueAngle
Purpose     : Get the rotation angle of sensor (compare with x, y, z axis) in degre
Parameters  : PACC_DATA_ANGLE - pointer to a struct store angle rotation
Return      : NULL
--------------------------------------------------------------------------------*/

void PMU6050_GetRotationAngle(ACC_DATA_RAW raw, void *pRotationAngle)
{

        /* Caculate the angle rotation */
        /* 180/PI = 57.296 */
        /*Fix: use atan2 -> result in -pi -> pi */
        float x_angle = 57.296 * atan2((float)raw.y, sqrt((float)raw.z*(float)raw.z + (float)raw.x*(float)raw.x));
        if(x_angle < 0) x_angle += 360.0;

        float y_angle = 57.296 * atan2((float)raw.x, sqrt((float)raw.z*(float)raw.z + (float)raw.y*(float)raw.y));
        if(y_angle < 0) y_angle += 360.0;

        float z_angle = 57.296 * atan2((float)raw.z, sqrt(((float)raw.x*(float)raw.x + (float)raw.y*(float)raw.y)));
        if(x_angle < 0) x_angle += 360.0;

        ((PANGLE)pRotationAngle)->x = x_angle;
        ((PANGLE)pRotationAngle)->y = y_angle;
        ((PANGLE)pRotationAngle)->z = z_angle;
}

/*--------------------------------------------------------------------------------
Function    : MPU6050_GetGyroValueRaw
Purpose     : Get raw value x, y, z of Gyro
Parameters  : PGYRO_DATA_RAW - pointer to struct store Gyro data
Return      : NULL
--------------------------------------------------------------------------------*/
void MPU6050_GetGyroValueRaw(GYRO_DATA_RAW *pValue)
{
        uint8_t pBuff[6];
        I2C_ReadData(pBuff, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 6);

        pValue->x = (int16_t)(CONVERT_TO_16BIT(pBuff[0], pBuff[1]) - Gyro_OffsetValueX);
        pValue->y = (int16_t)(CONVERT_TO_16BIT(pBuff[2], pBuff[3]) - Gyro_OffsetValueY);
        pValue->z = (int16_t)(CONVERT_TO_16BIT(pBuff[4], pBuff[5]) - Gyro_OffsetValueZ);

}

/*--------------------------------------------------------------------------------
Function    : PMU6050_GetGyroValueAngle
Purpose     : Get value x, y, z of Gyro
Parameters  : PGYRO_DATA_ANGLE - pointer to struct store Gyro data
Return      : NULL
--------------------------------------------------------------------------------*/
void PMU6050_GetGyroValueAngle(void *pValue)
{
        ;
}

/*--------------------------------------------------------------------------------
Function    : PMU6050_GyroConvertData
Purpose     : Convert data to m/s^2 by divide to scale
Parameters  : GYRO_DATA_RAW rawValue, void *scaledData
Return      : NULL
--------------------------------------------------------------------------------*/
void PMU6050_GyroConvertData(GYRO_DATA_RAW rawValue)
{
    GYRO_DATA_SCALED scaledData;
        scaledData.x = (float)rawValue.x / Gyro_scaleValue;
        scaledData.y = (float)rawValue.y / Gyro_scaleValue;
        scaledData.z = (float)rawValue.z / Gyro_scaleValue;
        SerialPrint( " gx: "); SerialPrintFloat(scaledData.x, 2);
        SerialPrint( " gy: "); SerialPrintFloat(scaledData.y, 2);
        SerialPrint( " gz: "); SerialPrintlnFloat(scaledData.z, 2);
}

/*--------------------------------------------------------------------------------
Function    :  MPU6050_Calibrate_Gyro
Purpose     :  Get the value to calibrate Gyro
Parameters  : NULL
Return      : NULL
--------------------------------------------------------------------------------*/

void MPU6050_Calibrate_Gyro(void)
{
        int i;
        int x = 0;
        int y = 0;
        int z = 0;
        uint8_t pBuff[6];

        Gyro_OffsetValueX = 0;
        Gyro_OffsetValueY = 0;
        Gyro_OffsetValueZ = 0;
        for(i = 0; i < 2000; i++)
        {

                I2C_ReadData(pBuff, MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H, 6);

                x = (int16_t)(CONVERT_TO_16BIT(pBuff[0], pBuff[1]));
                y = (int16_t)(CONVERT_TO_16BIT(pBuff[2], pBuff[3]));
                z = (int16_t)(CONVERT_TO_16BIT(pBuff[4], pBuff[5]));

                Gyro_OffsetValueX = (Gyro_OffsetValueX + x) >> 1;
                Gyro_OffsetValueY = (Gyro_OffsetValueY + y) >> 1;
                Gyro_OffsetValueZ = (Gyro_OffsetValueZ + z) >> 1;
        }
//        Gyro_OffsetValueX = (int16_t)(SumGyroX/200);
//        Gyro_OffsetValueY = (int16_t)(SumGyroY/200);
//        Gyro_OffsetValueZ = (int16_t)(SumGyroZ/200);
}




/*--------------------------------------------------------------------------------
Function    : ComplementaryFilter
Purpose     : Calculate angle use both acc and gyro
Parameters  : Raw data (ADC 16bit) of acc and gyro
Return      : ANGLE
--------------------------------------------------------------------------------*/

void Complementary_Filter(ACC_DATA_RAW accData, GYRO_DATA_RAW gyroData, void *pAngle)
{
        float pitchAcc, rollAcc;

        /* Angle around the X axis */
        ((PANGLE)pAngle)->x += ((float)gyroData.x / Gyro_scaleValue)*dt; // dt define as 0.015s
        /* Angle around the Y axis*/
        ((PANGLE)pAngle)->y += ((float)gyroData.y / Gyro_scaleValue)*dt;

        // Compensate for drift with accelerometer data if !bullshit
        // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
        long forceMagnitudeApprox = ABS(accData.x) + ABS(accData.y) + ABS(accData.z);
        if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
        {
                // Turning around the X axis results in a vector on the Y-axis
                pitchAcc = 57.296 * atan2((float)accData.y, sqrt((float)accData.z*(float)accData.z + (float)accData.x*(float)accData.x));
                ((PANGLE)pAngle)->x = ((PANGLE)pAngle)->x * 0.98 + pitchAcc * 0.02;

                // Turning around the Y axis results in a vector on the X-axis
                rollAcc = 57.296 * atan2((float)accData.x, sqrt((float)accData.z*(float)accData.z + (float)accData.y*(float)accData.y));
                ((PANGLE)pAngle)->y = ((PANGLE)pAngle)->y* 0.98 + rollAcc * 0.02;
        }

}


void readErrorCode(int eCode)
{
    if(eCode != 0)
    {
        SerialPrint("MPU6050 errorCode: ");
        SerialPrintlnS16(eCode);
    }
}



void printAccValue(ACC_DATA_RAW *accData, GYRO_DATA_RAW *gyData)
{
    if(accData)
    {
        SerialPrint(":ax: ");SerialPrintS16(accData ->x);
        SerialPrint(" :ay: ");SerialPrintS16(accData ->y);
        SerialPrint(" :az: ");SerialPrintS16(accData ->z);
    }
    if(gyData)
    {
        SerialPrint(":gx: ");SerialPrintS16(gyData ->x);
        SerialPrint(" :gy: ");SerialPrintS16(gyData ->y);
        SerialPrint(" :gz: ");SerialPrintlnS16(gyData ->z);
    }

}


void gyroPowerDown(void)
{
    uint8_t regValue;
    I2C_ReadData(&regValue, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,1);
    if((regValue & 0b01000000) == 0)
    {
        I2C_WriteByte(0b01000000, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1);
    }
}


void gyroPowerUP(void)
{
    uint8_t regValue;
    I2C_ReadData(&regValue, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,1);
    if((regValue & 0b01000000))
    {
        I2C_WriteByte(CLKSEL_0, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1);
    }
}
