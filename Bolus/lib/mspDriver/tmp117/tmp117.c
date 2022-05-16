#include "tmp117.h"

void startTmpConv(void);

void tmp117_init(void){
    i2c_init();
//    uint16_t response = (uint16_t)read_reg(0x48,TMP117_CONFIGURATION);
//    response &= 0xF3FF;
//    write_reg(TMP117_ADDR,TMP117_CONFIGURATION, (int)(response | (CONTINUOUS_CONVERSION_MODE <<10)));
//    startTmpConv();
    SerialPrint("Device ID: ");
    int response = 0;
    response = read_reg(TMP117_ADDR, TMP117_DEVICE_ID);
    SerialPrintlnU16(response);
    startTmpConv();
}

void startTmpConv(void)
{
//    SerialPrint("TMP117 Config: ");
//    int response = 0;
//    response = read_reg(TMP117_ADDR, TMP117_CONFIGURATION);
//    SerialPrintlnU16(response);
//    response &= 0xF01F;
//    response |= 0x0CA0;
//    writeConfig (response);


    SerialPrint("TMP117 Config: ");
    int response = 0;
    response = read_reg(TMP117_ADDR, TMP117_CONFIGURATION);
    SerialPrintU16(response);
    SerialPrint(" | ");
    response &= ~((0b11<<10)|(0b111<<7) | (0b11<<5));
    response |= ((ONE_SHOT_MODE & 0x03)<<10) | ((AVERAGE_CONVERSION_TIME & 0x07)<<7) |((AVERAGE_COUNT & 0x03)<<5);

//    response |= ((AVERAGE_CONVERSION_TIME & 0x07)<<7);
//    response |= ((AVERAGE_COUNT & 0x03)<<5);
    SerialPrintU16(response);
    writeConfig(response);
    SerialPrint(" | ");
    response = read_reg(TMP117_ADDR, TMP117_CONFIGURATION);
    SerialPrintlnU16(response);
}

int tmp117_gettemp(void)
{
    int tmp117_raw = 0;
    tmp117_raw = read_reg(TMP117_ADDR, 0);

//    SerialPrint("\r\nTMP RAW value: ");
    if(tmp117_raw<0)
    {
        tmp117_raw =(~tmp117_raw);
        tmp117_raw +=1;
    }
    startTmpConv();
    uint16_t response = 0;
    while(!(response & 0x2000))
    {
        response = read_reg(TMP117_ADDR, TMP117_CONFIGURATION);
    }
//    tmp117_shutdown();

//    SerialPrintS16(tmp117_raw);
//    tmp117_raw >>= 4;
//
//    if(tmp117_raw <= 0x7ff){
//
//        t= tmp117_raw * 6.25;
//
//    }
//    if(tmp117_raw>=0xc90) {
//        tmp117_raw = ~tmp117_raw;
//        tmp117_raw &=0x0fff;
//        tmp117_raw +=1;
//        t= tmp117_raw*6.25;
//        t =-t;
//    }
    return (int)tmp117_raw;
}

int read_reg(int address, int reg){
    int err;
    char buf[2] = {reg};
    err = i2c_start2(address, 0);
    if(err != 0) return err;
    err = i2c_senddata(buf, 1);
    if(err != 0) return err;
    err = i2c_stop();
    if(err != 0) return err;
    __delay_cycles(10000);
    err = i2c_start2(address, 1);
    if(err != 0) return err;
    err = i2c_readdata(buf, 2);
    if(err != 0) return err;

    int value = ((buf[0]<<8) | buf[1]);
    return value;

}

int write_reg(int address, int reg, int value){
    char buf[3];
    int err;
    buf[0] = (char)(reg & 0x0F);
    buf[1] = (char)((value>>8) & 0xFF);
    buf[2] = (char)(value & 0xFF);
    err = i2c_start2(address, 0);
    if(err != 0) return err;
    err = i2c_senddata(&buf[0], 3);
    if(err != 0) return err;
    err = i2c_stop();
    if(err != 0) return err;

    return 0;
}

void tmp117_shutdown()
{
    uint16_t response = (uint16_t)read_reg(TMP117_ADDR,TMP117_CONFIGURATION);
    response &= 0xF3FF;
    writeConfig(response | (SHUTDOWN_MODE <<10));
}

bool isEepromBusy (void) {
  // Bit 14 indicates the busy state of the eeprom
    uint16_t code = (uint16_t)read_reg (TMP117_ADDR, TMP117_EEPROM_UL );
    return (bool)((code >> 14) & 0x01);
}


void unlockEEPROM (void) {
// set bit 15
    uint16_t code = 0;
    code |= 1UL << 15;
    if(write_reg (TMP117_ADDR, TMP117_EEPROM_UL, code ))
    {
        SerialPrintln("Data Write Failed.");
    }
    delay(100);
}

void lockEEPROM (void) {
    // clear bit 15
    uint16_t code = 0;
    code &= ~(1UL << 15);
    write_reg (TMP117_ADDR, TMP117_EEPROM_UL, code );
    delay(100);
}

void generalCall(void)
{
    char buf = 0x06;
    i2c_start2(0x00,0);
    i2c_senddata(&buf, 1);
    i2c_stop();
    delay(100);
}

void writeConfig (uint16_t config_data) {
  if (!isEepromBusy()) {
    unlockEEPROM();
    write_reg (TMP117_ADDR, TMP117_CONFIGURATION, config_data);
    delay(100);
    while(isEepromBusy());
//    lockEEPROM();
    generalCall();
    delay(10);
    while(isEepromBusy());

  }
  else {
    SerialPrintln("EEPROM is busy");
  }
}
