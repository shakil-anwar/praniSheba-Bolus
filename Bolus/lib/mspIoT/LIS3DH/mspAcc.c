#include "mspAcc.h"
//#include "mspFlash.h"

void acc_begin(void)
{
    //Enable ACC port and set to high
    ACC_PORT_SET();
    ACC_DISABLE();

#ifndef SPI_INITIALIZED
#define SPI_INITIALIZED
    spi_begin(10000);//begin SPI
#endif
    // Disable FiFo and interrupt
    acc_write_register(LIS3DH_FIFO_CTRL_REG,BYPASS_MODE);

    //Enable Axis and Data Rate
#if defined(LOW_POWER_MODE)
    acc_write_register(LIS3DH_CTRL_REG1,
                       LIS3DH_DATA_RATE     |
                       LIS3DH_ENA_LOW_POW_MODE |
                       LIS3DH_ENABLE_Z_AXIS |
                       LIS3DH_ENABLE_Y_AXIS |
                       LIS3DH_ENABLE_X_AXIS );
    //setting accelerometer range
    acc_write_register(LIS3DH_CTRL_REG4, BLOCK_DATA_UPDATE | HIGH_RES_DISABLE);     //+-2g

#else
    acc_write_register(LIS3DH_CTRL_REG1,
                           LIS3DH_DATA_RATE     |
                           LIS3DH_ENABLE_Z_AXIS |
                           LIS3DH_ENABLE_Y_AXIS |
                           LIS3DH_ENABLE_X_AXIS );
    //setting accelerometer range
    acc_write_register(LIS3DH_CTRL_REG4, BLOCK_DATA_UPDATE | HIGH_RES_ENABLE);
#endif
    //setting interrupt
    acc_write_register(LIS3DH_CTRL_REG3, ENABLE_INTERRUPTS_ON_INT1);
    //setting accelerometer range
}

bool acc_is_connected()
{
//    if(acc_read_register(LIS3DH_WHO_AM_I) == LIS3DH_DEVICE_ID){
//        return 1;
//    }else{
//        return 0;
//    }
    return (acc_read_register(LIS3DH_WHO_AM_I) == LIS3DH_DEVICE_ID);
}

void acc_fifo_begin(void)
{
    uint8_t response;
    response = acc_read_register(LIS3DH_CTRL_REG5);
    acc_write_register(LIS3DH_CTRL_REG5, (response | LIS3DH_SET_FIFO_ENA_STATUS ));
    acc_write_register(LIS3DH_FIFO_CTRL_REG, FIFO_BUFFER_MODE  |
                                             TRIG_INT_ON_INT1 |GEN_WTM_INT_AFTER_BYTE );
}

unsigned int acc_readRawAccelX(void){
    return acc_read_register16(LIS3DH_OUT_X_L,2);
}

unsigned int acc_readRawAccelY(void){
    return acc_read_register16(LIS3DH_OUT_Y_L,2);
}
unsigned int acc_readRawAccelZ(void){
    return acc_read_register16(LIS3DH_OUT_Z_L,2);
}
uint8_t acc_get_fifo_status(void)
{
  return  acc_read_register(LIS3DH_FIFO_SRC_REG);
}

void acc_clear_fifo(void)
{
#if defined(MSPFLASHMEMORY_MSPFLASH_H_)
//    flashHold();
#endif

//    SerialPrint("\r\nInterrupt triggerred");
    SerialPrint("\r\n ACC FIFO Status: ");
    SerialPrintU8(acc_get_fifo_status() & 0x1F);
    uint8_t buffer[6];
    while( (acc_get_fifo_status() & FIFO_EMPTY_MASK) == 0 )
    {
        acc_get_xyz(buffer);
//        print_acc_xyx(acc_get_xyz(buffer));
    }
#if defined(MSPFLASHMEMORY_MSPFLASH_H_)
//    flashHoldRelease();
#endif
}
void print_acc_xyx(uint8_t *buffer6)
{
    SerialPrint("\r\nX : ");SerialPrintS8(buffer6[1]);
    SerialPrint(" | Y : "); SerialPrintS8(buffer6[3]);
    SerialPrint(" | Z : "); SerialPrintS8(buffer6[5]);
}

void acc_fifoStartRec(void)
{
    uint8_t response;
    response = acc_read_register(LIS3DH_FIFO_CTRL_REG);
    acc_write_register(LIS3DH_FIFO_CTRL_REG, response & 0x3F);
    response = acc_read_register(LIS3DH_FIFO_CTRL_REG);
    acc_write_register(LIS3DH_FIFO_CTRL_REG, (response & 0x3F)| FIFO_BUFFER_MODE);
}

void acc_shutDown(void)
{
    uint8_t response;
    response = acc_read_register(LIS3DH_CTRL_REG1);
//    response &= 0x0F;
    acc_write_register(LIS3DH_CTRL_REG1, response & 0x0F);
//    response = acc_read_register(LIS3DH_CTRL_REG1);
//    SerialPrint("\r\nACC CTRL REG1 : ");SerialPrintU8(response);
}


void en_acc_int(void)
{
#if defined(ACC_PORT_DIR)

#if defined(__MSP430G2553__)
    P2DIR &= ~(ACC_INT1_PIN | ACC_INT2_PIN);            // set INT1 & INT2 as input
    P2OUT &=  ~(ACC_INT1_PIN | ACC_INT2_PIN);           // set INT1 & INT2 pulled high
    P2REN |=  (ACC_INT1_PIN | ACC_INT2_PIN);              //
    P2IES &=  ~(ACC_INT1_PIN | ACC_INT2_PIN);           // set INT1 & INT2 on rising edge
    P2IFG &= ~(ACC_INT1_PIN | ACC_INT2_PIN);            // clear interrupt on INT1 and IN2
    P2IE  |=  (ACC_INT1_PIN | ACC_INT2_PIN);              // enable INT1 & INT2 on Port2
#elif defined(__MSP430FR2433__)
    P1DIR &= ~(ACC_INT1_PIN);            // set INT1 & INT2 as input
    P1OUT &=  ~(ACC_INT1_PIN);           // set INT1 & INT2 pulled high
    P1REN |=  (ACC_INT1_PIN);              //
    P1IES &=  ~(ACC_INT1_PIN);           // set INT1 & INT2 on rising edge edge
    P1IFG &= ~(ACC_INT1_PIN);            // clear interrupt on INT1 and IN2
    P1IE  |=  (ACC_INT1_PIN);              // enable INT1 & INT2 on Port2

#endif
// #elif defined(ACC_PORT_DIR == P1DIR)
//     P1DIR &= ~(ACC_INT1_PIN | ACC_INT2_PIN);            // set INT1 & INT2 as input
//     P1OUT &=  ~(ACC_INT1_PIN | ACC_INT2_PIN);           // set INT1 & INT2 pulled high
//     P1REN |=  (ACC_INT1_PIN | ACC_INT2_PIN);              //
//     P1IES &=  ~(ACC_INT1_PIN | ACC_INT2_PIN);           // set INT1 & INT2 on falling edge
//     P1IFG &= ~(ACC_INT1_PIN | ACC_INT2_PIN);            // clear interrupt on INT1 and IN2
//     P1IE  |=  (ACC_INT1_PIN | ACC_INT2_PIN);              // enable INT1 & INT2 on Port2
#endif
    _EINT();                            // Enable interrupts (set GIE in SR)
}
