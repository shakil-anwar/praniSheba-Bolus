/*
 * tmp117_register.h
 *
 *  Created on: Feb 4, 2021
 *      Author: Asus
 */

#ifndef TMP117_TMP117_REGISTER_H_
#define TMP117_TMP117_REGISTER_H_

/*
  TMP117 Registers as defined in Table 3 from datasheet (pg 24)
  Features of the TMP117:
   - ±0.1°C (Maximum) From –20°C to +50°C
   - ±0.15°C (Maximum) From –40°C to +70°C
   - ±0.2°C (Maximum) From –40°C to +100°C
   - ±0.25°C (Maximum) From –55°C to +125°C
   - ±0.3°C (Maximum) From –55°C to +150°C
   - Low Power Consumption 3.5-µA, 1-Hz Conversion Cycle
*/

// Address of the registers. This can be found on page 23 of the datasheet
enum TMP117_Register
{
  TMP117_TEMP_RESULT = 0X00,
  TMP117_CONFIGURATION = 0x01,
  TMP117_T_HIGH_LIMIT = 0X02,
  TMP117_T_LOW_LIMIT = 0X03,
  TMP117_EEPROM_UL = 0X04,
  TMP117_EEPROM1 = 0X05,
  TMP117_EEPROM2 = 0X06,
  TMP117_TEMP_OFFSET = 0X07,
  TMP117_EEPROM3 = 0X08,
  TMP117_DEVICE_ID = 0X0F
};



#endif /* TMP117_TMP117_REGISTER_H_ */
