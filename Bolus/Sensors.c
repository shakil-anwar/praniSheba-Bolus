/*
 * Sensors.c
 *
 *  Created on: Dec 14, 2020
 *      Author: sshuv
 */
#include "Sensors.h"
#include "./lib/mspIoT/mspIoT.h"
#include "Schema.h"
#include "Param.h"
#include "memory.h"

#define ACC_MAX_POINT 32
uint8_t ptrIndex = 0;
const uint8_t dataFreq = 5;
static uint8_t dataCounter = 0;
static bool updateRamPtr;
volatile uint32_t lastAccReadTime;
sensorStatus_t sensorStatus;

void readAcc(bolus_t *bolus);
void accIrq(void);
void memqSave();

void flashSave();
void readXyz();
void readXyz10Hz();
void readTemp();

static bolus_t *bolusLogPtr;

static bolus_t *bolusPtr = NULL;
static bool samplePtr;
static volatile uint8_t sampleCounter = 0;
static volatile uint8_t dataIndex = 0;
volatile bool accDataReady = 0;

#if defined(DEVICE_HAS_TEMP_SENSOR)
bool tempPacketReady;
static volatile bolusTemp_t tempBuf;
static volatile bolusTemp_t tempSaveBuf;
#endif
//bool _nowSend = false;

void sensorsBegin()
{
    acc_begin();
    acc_fifo_begin();
    acc_clear_fifo();
    ptrIndex = 0;
    bolusLogPtr = NULL;
    lastAccReadTime = second();
    updateRamPtr = false;
    samplePtr = true;
    accDataReady = false;
    sampleCounter = 0;
    dataIndex  = 0;

#if defined(DEVICE_HAS_TEMP_SENSOR)
    tempBuf.header.type = 2;
    tempBuf.header.id = DEVICE_ID;
    tempPacketReady = false;
#endif
}

void sensorStart()
{
#if defined(BOARD_MSP430G2_V010)
    attachInterrupt2(3, RISING, accIrq);
//    attachInterrupt2(4,RISING,accIrq);
#elif defined(BOARD_MSP430FR_V010)
    attachInterrupt1(1,RISING,accIrq);
    sensorStatus = IDLE;
//    attachInterrupt1(1,RISING,acc_clear_fifo);
#endif
}

void accIrq(void)
{
//    SerialPrint("ACC TRIG: ");
//    SerialPrintlnU32(second());
//    bolus_t *bolusPtr = ramQHead();
//    readAcc(bolusPtr);
//    readXyz();
    readXyz10Hz();
    SerialPrintln("..");
//    readXyz10Hz();
    if((sensorStatus == READING_DATA) || (sensorStatus == SAVING_DATA))
    {
        sensorStatus = IDLE;
    }

}

void flashSave()
{
    if(sensorStatus != SENDING_DATA)
    {
        sensorStatus = SAVING_DATA;
        flashReleasePowerDown();
        uint8_t *ramqTail = NULL;
        ramqTail = ramQReadTail();
#if defined(SHOW_DEBUG)
        SerialPrintln("-->Saving to flash");
#endif
        while(ramqTail !=NULL)
        {
            memqWrite(&memq, ramqTail);
            ramqTail = ramQReadTail();
#if defined(SHOW_DEBUG)
            memqPrintLog(&memq);
#endif
            //shut down data sampling if flash locked.
        }
        flashPowerDown();
    }
}

int readOutAccFifo()
{
        if((acc_get_fifo_status() & 0x1F) >= 31)
        {
            readXyz10Hz();
            sensorStatus = IDLE;
            return 1;
        }
    return 0;
}


void readXyz10Hz()
{

    static volatile uint32_t accTime = 0;
    uint8_t xyzBuffer[6];
    dataCounter++;

    int accData = 0;
    if(sensorStatus != SENDING_DATA)
    {
        for(accData = 0 ; accData<32; accData++)
        {
            sensorStatus = READING_DATA;
            acc_get_xyz (xyzBuffer);
            if(updateRamPtr == false)
            {
                if(bolusPtr)
                {
                    bolusPtr -> header.checksum = 0;
                    bolusPtr -> header.checksum = checksumCalc((uint8_t*)bolusPtr, sizeof(struct bolus_t));
                }

                ramqUpdate();

#if defined(DEVICE_HAS_TEMP_SENSOR)
                if(tempPacketReady)
                {
                    if(ramqIsLocked())
                    {
                        flashSave();
                    }
                    uint8_t *tempPtr = (uint8_t *)ramqGetNextHead();
                    memcpy(tempPtr,(uint8_t *)&tempSaveBuf,sizeof(struct bolusTemp_t));
                    ramqUpdate();
                    tempPacketReady = false;
                }
#endif
                bolusPtr = (bolus_t*)ramqGetNextHead();

                if(ramqIsLocked())
                {
                   //store data in flash
                    flashSave();
                    bolusPtr = (bolus_t*)ramqGetNextHead();
                }
                updateRamPtr = true;

                if(bolusPtr)
                {
    //                ramqPrintLog();
                    bolusLogPtr = bolusPtr;
                    if(abs(second() - accTime) > 60)
                    {
                        accTime = second() - 4;
                    }
                    accTime +=4;
                    SerialPrint("Acc time: "); SerialPrintlnU32(accTime);
                    bolusPtr -> header.type = 1;
                    bolusPtr -> header.id   = DEVICE_ID;
                    bolusPtr -> unixTime    = accTime;
                    lastAccReadTime = accTime;
                }

            }

            if(dataIndex == 0)
            {
                if(bolusPtr != NULL)
                {
                    bolusPtr->x[sampleCounter] = xyzBuffer[1];
                    bolusPtr->y[sampleCounter] = xyzBuffer[3];
                    bolusPtr->z[sampleCounter] = xyzBuffer[5];
                    sampleCounter++;
                    samplePtr = true;
                }
            }
            dataIndex++;

            if((dataIndex % dataFreq) == 0)
            {

//                SerialPrint("dataIndex: ");
//                SerialPrintlnU8(dataIndex);
                dataIndex = 0;

            }

            if(samplePtr == true)
            {
                if((sampleCounter % BOLUS_SAMPLE_IN_PACKET) == 0 )
                {
//                    SerialPrint("sampleCounter: ");
//                    SerialPrintlnU8(sampleCounter);
                    sampleCounter = 0;
                    updateRamPtr = false;
                    samplePtr = false;
                }
            }
        }
    }
    else
    {
        accDataReady = true;
    }


}


void readXyz()
{
#if defined(DEVICE_HAS_TMP117)
    readTemp();
#endif
    uint8_t xyzBuffer[6];
    dataCounter++;
    if((dataCounter % dataFreq) == 0)
    {


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
        dataCounter = 0;
    }
    else
    {
        int accData = 0;
        for(accData = 0 ; accData<32; accData++)
        {
            acc_get_xyz (xyzBuffer);
        }
    }
}

void readTemp()
{
    static volatile int i= 0;
#if defined(DEVICE_HAS_TEMP_SENSOR)
    if(i==0)
    {
        tempBuf.header.type = 2;
        tempBuf.header.id = DEVICE_ID;
    }
#endif

#if defined(DEVICE_HAS_TMP117)
    tempBuf.tempData[i].unixTime = second();
    tempBuf.tempData[i].temp = tmp117_gettemp();
#elif defined(DEVICE_HAS_DS18B20)
    tempBuf.tempData[i].unixTime = second();
    tempBuf.tempData[i].temp = (int)ds18b20_get_temp();
    SerialPrint("Temperature: ");
    SerialPrintS16(i);
    SerialPrint(" ");
    SerialPrintlnFloat((float)tempBuf.tempData[i].temp,2);
#endif

#if defined(DEVICE_HAS_TEMP_SENSOR)
    i++;
//    printBuffer((uint8_t *)&tempBuf, 32);
    if(i>3)
    {
        if(sensorStatus == IDLE)
        {
            if(!memqIsLock(&memq))
            {
                SerialPrint("Saving T");

//                flashReleasePowerDown();
//                delay(5);
                tempBuf.header.checksum = 0;
                tempBuf.header.checksum = checksumCalc((uint8_t*)&tempBuf, sizeof(struct bolusTemp_t));
                memcpy((uint8_t *)&tempSaveBuf,(uint8_t *)&tempBuf,sizeof(struct bolusTemp_t));
                tempPacketReady = true;
                printBuffer((uint8_t *)&tempSaveBuf, 32);
//                memqWrite(&memq, (uint8_t *)&tempBuf);
//                flashPowerDown();
            }
        }

        i = 0;
    }
    delay(1);
#endif
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
//    uint16_t i;


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

