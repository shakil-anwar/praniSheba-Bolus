/*
 * Schema.h
 *
 *  Created on: Dec 14, 2020
 *      Author: sshuv
 */

#ifndef SCHEMA_H_
#define SCHEMA_H_
#include "mspDriver.h"
#include "mspIoT.h"
#include "Param.h"


struct header_t
{
  uint8_t type;
  uint8_t checksum;
  uint16_t id;
};


/****************************Payload Schema*****************/
typedef struct bolus_t
{
    //Header
    struct header_t header;
    //payload
    uint32_t unixTime;
    uint8_t x[BOLUS_SAMPLE_IN_PACKET];
    uint8_t y[BOLUS_SAMPLE_IN_PACKET];
    uint8_t z[BOLUS_SAMPLE_IN_PACKET];
}bolus_t;


typedef union payload_t
{
  bolus_t bolus;
}payload_t;







/*************************Query Schema*****************************/

typedef struct confPacket_t
{
  uint8_t type;
  uint8_t opCode;
  uint8_t txAddrByte;
  uint8_t padding;
  uint32_t uTime;
}confPacket_t;

typedef struct shedulePacket_t
{
  uint8_t type; //device type
  uint8_t opCode;//device op code
  uint8_t slotNo;
  uint32_t unixTime;

}shedulePacket_t;

typedef union queryData_t
{
   confPacket_t confPacket;
   shedulePacket_t schedulePacket;
}queryData_t;

void schemaBegin();
uint8_t checksumCalc(uint8_t *buf,uint8_t len);
void printBolusPacket(bolus_t *bolus);

extern volatile payload_t  payload[TOTAL_PAYLOAD_BUFFER];
extern queryData_t queryBuffer;

#endif /* SCHEMA_H_ */
