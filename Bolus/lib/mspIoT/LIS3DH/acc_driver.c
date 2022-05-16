#include "acc_driver.h"
//#include "lis3dh_register.h"

uint8_t ac_status;

uint8_t acc_read_register(uint8_t addr)
{
    uint16_t response;
    ACC_ENABLE();
    response = spi_transfer16((addr | LIS3DH_READ_MASK)<<8);
    ACC_DISABLE();
    return (uint8_t) (response);
}

uint8_t acc_write_register(uint8_t addr, uint8_t data)
{
    ACC_ENABLE();
    spi_transfer16((data & 0x00FF) | (addr << 8));
    ACC_DISABLE();
    return 0;
}

unsigned int acc_read_register16(uint8_t addr, uint8_t level)
{
    int i=0;
    unsigned int out;
    uint8_t output[2];
    ACC_ENABLE();
    spi_transfer(addr | 0x80 | 0x40);
//    output[0] = spi_transfer(0x00);
//    output[1] = spi_transfer(0x00);
    do{
        output[i] = spi_transfer(0x00);
        i++;
    }while(i<2);
    ACC_DISABLE();

    out = output[0];
    out |= (output[1] << 8);
    return out;
}

uint8_t *acc_get_xyz(uint8_t *buffer6)
{
    uint8_t *p = buffer6;
    uint8_t i;
    ACC_ENABLE();
    spi_transfer(LIS3D_READ_WITH_AUTO_INC | LIS3DH_OUT_X_L);
    for(i = 0; i< 6 ;i ++)
    {
        *p = spi_transfer(0x00);
         p++;
    }
    ACC_DISABLE();
    return buffer6;
}

