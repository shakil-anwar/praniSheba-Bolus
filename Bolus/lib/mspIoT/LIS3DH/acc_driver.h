#ifndef LIS3DH_DRIVER_H_
#define LIS3DH_DRIVER_H_
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "acc_usr_conf.h"
#include "acc_register.h"



#define ACC_PORT_SET() (ACC_PORT_DIR |= ACC_PIN)
#define ACC_ENABLE()   (ACC_PORT_OUT &= ~ACC_PIN)
#define ACC_DISABLE()  (ACC_PORT_OUT |= ACC_PIN)

extern uint8_t ac_status;

uint8_t acc_read_register(uint8_t addr);
uint8_t acc_write_register(uint8_t addr, uint8_t data);
unsigned int acc_read_register16(uint8_t addr, uint8_t level);
uint8_t *acc_get_xyz(uint8_t *buffer6);



#endif /* LIS3DH_DRIVER_H_ */
