/*
 * Schema.c
 *
 *  Created on: Dec 14, 2020
 *      Author: sshuv
 */
#include "Schema.h"


volatile payload_t payload[TOTAL_PAYLOAD_BUFFER];
queryData_t queryBuffer;

void schemaBegin()
{
    ramQSet(&payload, sizeof(payload_t), TOTAL_PAYLOAD_BUFFER);
}


uint8_t checksumCalc(uint8_t *buf,uint8_t len)
{
  struct header_t *hPtr = (struct header_t*)buf;
  hPtr->checksum = 0;

  uint16_t sum = 0;
  uint8_t *ptr = buf;
  uint8_t i;
  for(i = 0; i< len; i++)
  {
    sum += (uint16_t)ptr[i];
  }
  return (uint8_t)sum;
}



void printBolusPacket(bolus_t *bolus)
{
    SerialPrint("PLD->Type:");SerialPrintU8(bolus->header.type);
    SerialPrint("|id:");SerialPrintU16(bolus->header.id);
    SerialPrint("|uTime:");SerialPrintU32(bolus->unixTime);
    SerialPrint("|cSum:");SerialPrintU8(bolus->header.checksum);
    SerialPrint("|ramQ:");SerialPrintlnU8(ramQCounter);

    uint8_t i = 0;
    for(i = 0; i< BOLUS_SAMPLE_IN_PACKET; i++)
    {
        SerialPrint("X : ");SerialPrintS8(bolus->x[i]);
        SerialPrint(" | Y : "); SerialPrintS8(bolus->y[i]);
        SerialPrint(" | Z : "); SerialPrintlnS8(bolus->z[i]);
    }
}

