/*
 * Sensors.c
 *
 *  Created on: Dec 14, 2020
 *      Author: sshuv
 */
#include "Sensors.h"
#include "mspIoT.h"
#include "Schema.h"
#include "Param.h"
#include "memory.h"

#define ACC_MAX_POINT 32
uint8_t ptrIndex = 0;

void readAcc(bolus_t *bolus);
void accIrq(void);
void memqSave();

void flashSave();
void readXyz();

static bolus_t *bolusLogPtr;
//bool _nowSend = false;

void sensorsBegin()
{
    acc_begin();
    acc_fifo_begin();
    acc_clear_fifo();
    ptrIndex = 0;
    bolusLogPtr = NULL;
}

void sensorStart()
{
#if defined(BOARD_MSP430G2_V010)
    attachInterrupt2(3, RISING, accIrq);
//    attachInterrupt2(4,RISING,accIrq);
#elif defined(BOARD_MSP430FR_V010)
    attachInterrupt1(1,RISING,accIrq);
#endif
}

void accIrq(void)
{
    SerialPrint("ACC TRIG TIME : ");
    SerialPrintlnU32(second());
//    bolus_t *bolusPtr = ramQHead();
//    readAcc(bolusPtr);
    readXyz();

}


void flashSave()
{
    uint8_t *ramqTail = ramqGetTail();
    flashReleasePowerDown();
    SerialPrintln("----->Saving to flash");
    while(ramqTail !=NULL)
    {
        memqWrite(&memq, ramqTail);
        ramqTail = ramqGetTail();
        memqPrintLog(&memq);
        //shut down data sampling if flash locked.
    }
    flashPowerDown();
}

void readXyz()
{
    uint8_t xyzBuffer[6];

    uint32_t accTime = second();
    accTime -=32;

    bolus_t *bolusPtr;
    uint8_t packet;
    for(packet = 0; packet < 4; packet++)
    {
       bolusPtr = (bolus_t*)ramqGetHead();
       if(packet == 0)
       {
           bolusLogPtr = bolusPtr;
       }

        //populate sensor data
        uint8_t i = 0;
        for(i = 0; i < BOLUS_SAMPLE_IN_PACKET; i++)
        {
            acc_get_xyz (xyzBuffer);
            bolusPtr->x[i] = xyzBuffer[1];
            bolusPtr->y[i] = xyzBuffer[3];
            bolusPtr->z[i] = xyzBuffer[5];
        }
        //populate header
        bolusPtr -> header.type = 1;
        bolusPtr -> header.id   = DEVICE_ID;
        bolusPtr -> unixTime    = accTime;
        bolusPtr -> header.checksum = 0;
        bolusPtr -> header.checksum = checksumCalc((uint8_t*)bolusPtr, sizeof(struct bolus_t));

        //update time for next packet
        accTime +=8UL;
        //
        if(ramqIsLocked())
        {
           //store data in flash
            flashSave();
        }
        ramqPrintLog();
    }

}



void readAcc(bolus_t *bolus)
{
    bolus_t *bolusPtr = bolus;
    uint8_t xyzBuffer[6];
    uint32_t accSample = 0;
    ptrIndex = 0;
    uint32_t accTime = second();
    accTime -=32;

    bolusPtr -> header.type = 1;
    bolusPtr -> header.id   = DEVICE_ID;
    bolusPtr -> unixTime    = accTime;
    bolusPtr -> header.checksum = 0;

    bolusPtr -> header.checksum = checksumCalc((uint8_t*)bolusPtr, sizeof(struct bolus_t));
    do
    {
        acc_get_xyz (xyzBuffer);
//        print_acc_xyx(xyzBuffer);

        bolusPtr->x[ptrIndex] = xyzBuffer[1];
        bolusPtr->y[ptrIndex] = xyzBuffer[3];
        bolusPtr->z[ptrIndex] = xyzBuffer[5];

        ptrIndex++;
        if (ptrIndex >= BOLUS_SAMPLE_IN_PACKET)
        {
            bolusPtr -> header.checksum = checksumCalc((uint8_t*)bolusPtr, sizeof(struct bolus_t));
//            SerialPrintln("ramQ Counter: ");SerialPrintS8(ramQCounter);
            //send data via nrf
            bolusPtr = ramQUpdateHead();
            bolusPtr -> header.type = 1;
            bolusPtr -> header.id   = DEVICE_ID;
            accTime +=8;
            bolusPtr -> unixTime    = accTime;
            bolusPtr -> header.checksum = 0;
            ptrIndex = 0;
        }
    }
    while (++accSample < ACC_MAX_POINT);
    bolusPtr -> header.checksum = checksumCalc((uint8_t*)bolusPtr, sizeof(struct bolus_t));
    memqSave();
//    _nowSend = true;
//    nrfStandby1();
//    radioSendSM();
//    nrfPowerDown();
}

void sensorPrintLog()
{
    static bolus_t *bolusLastLogPtr = NULL;
    if(bolusLogPtr !=bolusLastLogPtr)
    {
        uint8_t i = 0;
        for(i = 0; i< 4; i++)
        {
           printBolusPacket(&bolusLogPtr[i]);
        }
        bolusLastLogPtr = bolusLogPtr;
    }
}


void memqSave()
{
  //When ramq full, _ramQBase points to the base address,

  if (_ramQBase != NULL)
  {
    SerialPrintln("----->Saving to flash");
    flashReleasePowerDown();
    uint8_t *ramqPtr = ramQRead();
    uint16_t i;


    while (ramqPtr)
    {
        memqWrite(&memq, ramqPtr);
      //uint32_t curPage = (memq ->ringPtr._head) >> 8;
      //flash.dumpPage(curPage, pageBuf);
      //Serial.print(F("Counter : ")); Serial.println(memq -> _ptrEventCounter);
//      ramqPtr += sizeof(payload_t);
      ramQUpdateTail();
      ramqPtr = ramQRead();
    }
    flashPowerDown();

    _ramQBase = NULL; //null pagePtr to avoid overwrite
  }
}

