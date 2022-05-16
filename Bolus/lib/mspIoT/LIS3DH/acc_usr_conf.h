#ifndef ACC_USR_CONF_H_
#define ACC_USR_CONF_H_

#include "../../mspDriver/mspDriver.h"
#include "acc_register.h"

/*********** ACC Pin Conf*************/
#if defined(__MSP430G2553__)
#define ACC_PORT_DIR     			P2DIR
#define ACC_PORT_OUT    			P2OUT
#define ACC_PIN          			BIT5
#define ACC_INT1_PIN                BIT3
#define ACC_INT2_PIN                BIT4
#elif defined(__MSP430FR2433__)
#define ACC_PORT_DIR                P2DIR
#define ACC_PORT_OUT                P2OUT
#define ACC_PIN                     BIT2
#define ACC_INT1_PIN                BIT1
//#define ACC_INT2_PIN                BIT4
#endif

#define LOW_POWER_MODE              //This macro sets ACC LOW power mode. to enable high resolution mode comment this
#define LIS3DH_DATA_RATE            DATA_RATE_10HZ

#define LIS3DH_SET_FIFO_ENA_STATUS  FIFO_ENA

#define ENABLE_INTERRUPTS_ON_INT1   (ENA_OVERRUN_INT1 | ENA_WATERMARK_INT1)

#define FIFO_BUFFER_MODE            STREAM_MODE
#define GEN_WTM_INT_AFTER_BYTE      31 //Acc generate wtm int after filling the defined byte
#define TRIG_INT_ON_INT1            0
#define TRIG_INT_ON_INT2            (1<<5)


//#define ACC_INT_DEBUG_ON

#endif

