/*
 * lis3dh_register.h
 *
 *  Created on: Sep 3, 2020
 *      Author: Asus
 */

#ifndef LIS3DH_REGISTER_H_
#define LIS3DH_REGISTER_H_

/*****************Instruction***************/
#define LIS3DH_READ_MASK              0x80
#define LIS3DH_WRITE_MASK             0x00
#define LIS3DH_ADDR_AUTO_INC          0x40
#define LIS3D_READ_WITH_AUTO_INC      (LIS3DH_READ_MASK | LIS3DH_ADDR_AUTO_INC)


#define LIS3DH_STATUS_REG_AUX         0x07
#define LIS3DH_OUT_ADC1_L             0x08
#define LIS3DH_OUT_ADC1_H             0x09
#define LIS3DH_OUT_ADC2_L             0x0A
#define LIS3DH_OUT_ADC2_H             0x0B
#define LIS3DH_OUT_ADC3_L             0x0C
#define LIS3DH_OUT_ADC3_H             0x0D
#define LIS3DH_INT_COUNTER_REG        0x0E
#define LIS3DH_WHO_AM_I               0x0F

#define LIS3DH_TEMP_CFG_REG           0x1F
#define LIS3DH_CTRL_REG1              0x20
#define LIS3DH_CTRL_REG2              0x21
#define LIS3DH_CTRL_REG3              0x22
#define LIS3DH_CTRL_REG4              0x23
#define LIS3DH_CTRL_REG5              0x24
#define LIS3DH_CTRL_REG6              0x25
#define LIS3DH_REFERENCE              0x26
#define LIS3DH_STATUS_REG2            0x27
#define LIS3DH_OUT_X_L                0x28
#define LIS3DH_OUT_X_H                0x29
#define LIS3DH_OUT_Y_L                0x2A
#define LIS3DH_OUT_Y_H                0x2B
#define LIS3DH_OUT_Z_L                0x2C
#define LIS3DH_OUT_Z_H                0x2D
#define LIS3DH_FIFO_CTRL_REG          0x2E
#define LIS3DH_FIFO_SRC_REG           0x2F
#define LIS3DH_INT1_CFG               0x30
#define LIS3DH_INT1_SRC               0x31
#define LIS3DH_INT1_THS               0x32
#define LIS3DH_INT1_DURATION          0x33

#define LIS3DH_CLICK_CFG              0x38
#define LIS3DH_CLICK_SRC              0x39
#define LIS3DH_CLICK_THS              0x3A
#define LIS3DH_TIME_LIMIT             0x3B
#define LIS3DH_TIME_LATENCY           0x3C
#define LIS3DH_TIME_WINDOW            0x3D

// Register Mask

#define LIS3DH_FIFO_EMPTY             0b00100000
#define LIS3DH_FIFO_WTM               0b10000000
#define LIS3DH_FIFO_OVRN_FIFO         0b01000000

/*****************Configuration Masking****************************/
/*****************WHO AM I *********************/
#define LIS3DH_DEVICE_ID			0b00110011

/*****************CTRL_REG1*********************/
#define LIS3DH_ENABLE_X_AXIS        (1<<0)
#define LIS3DH_ENABLE_Y_AXIS        (1<<1)
#define LIS3DH_ENABLE_Z_AXIS        (1<<2)
#define LIS3DH_ENA_LOW_POW_MODE     (1<<3)

/*****************ACC Data acquisition rate************************/
// 0x00=Power Down Mode; 0x01=1Hz; 0x02=10Hz; 0x03=25Hz; 0x04=50Hz; 0x05=100Hz; 0x06=200Hz; 0x07=400Hz; 0x08=Low Power Mode(1.6KHz); 0x09=HR/normal(1.344KHz)/Low-power mode (5.376KHz)
#define DATA_RATE_1HZ               (0b0001<<4)
#define DATA_RATE_10HZ              (0b0010<<4)
#define DATA_RATE_25Hz              (0b0011<<4)

/*************CTRL_REG4*************************/
#define BLOCK_DATA_UPDATE           (1<<7)
#define HIGH_RES_ENABLE             (1<<3)
#define HIGH_RES_DISABLE            (0<<3)

/*****************CTRL_REG5*********************/
#define FIFO_ENA                    (1<<6)
#define FIFO_DISABLE                (0<<6)
/**************FIFO_CTRL_REG********************/
#define BYPASS_MODE                 (0b00<<6)
#define FIFO_MODE                   (0b01<<6)
#define STREAM_MODE                 (0b10<<6)
#define STREAM_TO_FIFO              (0b11<<6)

/**************FIFO_SRC_REG********************/
#define FIFO_EMPTY_MASK				0x20

/**************Interrupt Mode******************/
#define ENA_OVERRUN_INT1            (1<<1)
#define ENA_WATERMARK_INT1          (1<<2)

#endif /* LIS3DH_REGISTER_H_ */
