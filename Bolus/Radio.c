/*
 * Radio.c
 *
 *  Created on: Dec 14, 2020
 *      Author: sshuv
 */
#include "Radio.h"

#include "Schema.h"
#include "Param.h"
#include "Pin.h"
#include "memory.h"
#include "All.h"
#include "utility.h"

#define QUERY_PIPE 0

struct pldReadInfo_t
{
    uint32_t flashAvail;
    uint32_t ramAvail;
    bool isOldPkt;
    bool isRamActive;
};



void txIsr(void);
void rxIsr(void);
void maxRtIsr(void);


uint8_t* memRead();
void rfSend(uint8_t *data);
int rfAckWait();
bool isBsConnected();
bool isMySlot();
void printPldReadInfo();
bool nrfSendLoop();

bool isPayloadReady;
bool ramReadFlag = true;
bool _radioStartXfer = false;

struct pldReadInfo_t pldObj;

nrfNodeConfig_t nrfConfig;
payload_t pldBuf;
uint8_t *pldPtr;

//uint8_t pipe0Addr[5] = {120, 124, 124, 124, 125};
//uint8_t pipe0Addr[5] = {101, 101, 102, 103, 104};
//uint8_t pipe0Addr[5] = {135,145,145,145,150};

//uint8_t pipe0Addr[5] = { 130,140,130,140,150};
uint8_t pipe0Addr[5] = { 201,202,203,204,205};

bool radioSetLowPower()
{
    nrfPowerDown();
    return true;
}

void radioBegin()
{
//    SerialPrint("Setting NRF");
    nrfSetTimers(millis, NULL);
    nrfSetPin(&NRF_CE_PORT, NRF_CE_PIN, &NRF_CSN_PORT, NRF_CSN_PIN);
    nrfSetIrqs(txIsr, rxIsr, maxRtIsr);
    nrfSetDebug(false);

    nrfQryObj.pipe = QUERY_PIPE;
    nrfQryObj.activePingAddr = pipe0Addr;
    nrfQueryBeginClient(&nrfQryObj);



    nrfBegin(SPEED_2MB, POWER_ZERO_DBM, SPI_SPEED);
    nrfPowerDown();

    SerialPrint("NRF Is Running: ");
    SerialPrintlnU8(nrfIsRunning());

    //begin radio transmission mechanism
    xferBegin(memRead, rfSend, rfAckWait, millis);
    xferConfig(1,true);
    pldPtr = NULL; //This has to be null for proper operation.
    isPayloadReady = true;
}

void radioStart()
{
#if defined(BOARD_MSP430G2_V010)
  attachInterrupt2(2, FALLING, nrfIrq);
#elif defined(BOARD_MSP430FR_V010)
    attachInterrupt1(NRF_IRQ, FALLING, nrfIrq);
#endif
    _enable_interrupt(); //Enable Global Interrupt
}

bool nrfSendLoop()
{
    static uint8_t *_ptr = NULL;
    static uint32_t _startMillis;
    static uint8_t  _totalTime;
    static int _ackCode;
    static volatile uint8_t _tryCount;

    uint16_t totalXfer = 0;
//    if(_debug)
//    {
//       _startMillis = _millis();
//    }

    do
    {
        if(accDataReady == true)
        {
            sensorStatus = IDLE;
            readOutAccFifo();
            accDataReady = false;
            sensorStatus = SENDING_DATA;
        }
        _ptr = memRead();
        if (_ptr != NULL)
        {
            // if(_debug)
            // {
            //     _startMillis = _millis();
            // }
            rfSend(_ptr);
            _ackCode = rfAckWait();
            if(_ackCode == 200)
            {
                isPayloadReady = true;
                totalXfer++;
                // if(_debug)
                // {
                //      _totalTime = _millis() -  _startMillis;
                //      SerialPrintF(P("XFER->OK|Time:"));
                //   SerialPrintlnU32(_totalTime);
                // }
            }
            else
            {
//                if(_debug) {SerialPrintF(P("XFER->NOK"));}
                isPayloadReady = false;
            }
        }

    }while(_ptr != NULL && isPayloadReady);

    if(totalXfer>0)
    {
        _totalTime = millis() -  _startMillis;
        SerialPrintF(P("XFER->TOTAL_PKT:"));
        SerialPrintU16(totalXfer);
        SerialPrintF(P("|Time:"));SerialPrintlnU32(_totalTime);
    }
    return isPayloadReady;
}

void txIsr(void)
{
//    nrfClearTxDs();
}

void rxIsr(void)
{
    //  SerialPrintln("RX Trig");
//    SerialPrint("RX IQR : ");
//    SerialPrintlnU16(_nrfIrqState);
//    nrfClearRxDr();
}

void maxRtIsr(void)
{
//    nrfClearMaxRt();
    nrf_flush_rx();
    nrf_flush_tx();
}




void printPldReadInfo()
{
    SerialPrint("RADIO->SEND->rAvail:");SerialPrintU32(pldObj.ramAvail);
    SerialPrint("|fAvail:");SerialPrintU32(pldObj.flashAvail);
    SerialPrint("|ramActv:");SerialPrintU8(pldObj.isRamActive);
    SerialPrint("|isOld:");SerialPrintlnU8(pldObj.isOldPkt);
}

uint8_t radioSendSM()
{
    uint8_t isOk = (uint8_t)isMySlot();
    saveUnixTime();

    uint32_t slotSec = calcNextSlotUnix(second(), &nrfConfig);
    rtcSetAlarm(slotSec,nrfConfig.momentDuration,alarmTask);

    if (isOk)
    {
        nrfTxSetModeClient(BS_DATA,&nrfConfig);
//        nrfTxReady(&nrfConfig);
        xferReady();


        flashReleasePowerDown();
        pldObj.flashAvail = memqAvailable(&memq);
        pldObj.ramAvail =  ramqAvailable();
        printPldReadInfo();
//        pldObj.ramAvail =
//        SerialPrint("====>Total Buf : ");
//        SerialPrintlnU32(packetAvail);

        if (pldObj.flashAvail > 0)
        {
//            ramReadFlag = false; //read from flash
//            pldObj.isFlashActive = true;
            pldObj.isRamActive = false;
        }
        else
        {
//            pldObj.isFlashActive = false;
            pldObj.isRamActive = true;
            flashPowerDown();
//            ramReadFlag = true;  //read from ram
        }

        if(xferSendLoopV3())
        {
            isOk = 1;
        }
        else
        {
            isOk = 2;
        }

//        flashPowerDown();
    }
    return isOk;
}
//nrfPowerDown();

uint8_t* memRead()
{
    if (pldPtr == NULL)
    {
        pldObj.isOldPkt = false;
        if (pldObj.isRamActive)
        {
//            SerialPrintln("Reading From Ram");
            pldPtr = ramQReadTail();
//            ramQUpdateTail();
        }
        else
        {
//            SerialPrintln("Reading From Flash");
            pldPtr = memqRead(&memq, (uint8_t*) &pldBuf);
            printBuffer(pldPtr,32);

            if (pldPtr == NULL)
            {
//                ramReadFlag = true;
                pldObj.isRamActive = true;
                pldPtr = ramQReadTail();
//                ramQUpdateTail();
                flashPowerDown();
//                SerialPrintln("-----------------Switching to Ram");
            }
        }

//        if (pldPtr != NULL)
//        {
//            printBuffer(pldPtr, sizeof(payload_t));
//        }
//        return pldPtr;
    }
    else
    {
        pldObj.isOldPkt = true;
        //    SerialPrintln("----->Read Mem : Old");
    }
//    printPldReadInfo();
    return pldPtr;
}

void rfSend(uint8_t *data)
{
//    SerialPrintln("Sending Via nrf");
    //  RF_LED_ON();
    nrfWrite(data, sizeof(payload_t));
    nrtTxStartTransmission();
    //  RF_LED_OFF();
}

int rfAckWait()
{
    // Serial.println(F("Ack wait"));
    bool res = nrfAck();
    if (res)
    {
        pldPtr = NULL;
        return 200;
    }
    return -1;
}

bool isBsConnected()
{
    int8_t tryCount = 3;
    do
    {
        if (nrfPing())
        {
            return true;
        }
        else
        {
            delay(50);
        }
    }
    while (--tryCount);
    return false;
}

bool isMySlot()
{
    int8_t tryCount = 1;
    uint32_t uTime = 0, slotSec;
    _nrfDebug = true;
    struct pong_t pong;

    do
    {
        uTime = nrfPingSlot(DEVICE_ID, nrfConfig.slotId, &pong);
        int32_t delayTime = (int32_t)second();
        delayTime = (int32_t)((uint32_t)delayTime-pong.second);

        if(uTime)
        {
            SerialPrint("delay Time: ");
            SerialPrintlnS32(delayTime);

            if(abs(delayTime)>1)
            {
                SerialPrint("abs delay Time: ");
                SerialPrintlnS32(delayTime);

                setSecond(pong.second);
            }
            if (pong.isConfigChanged != 1)
            {
                if(abs(delayTime) < nrfConfig.perNodeInterval)
                {
                    if(delayTime > 0)
                    {
                        if(delayTime <= 1)
                        {
                            delayTime = delayTime*1000;
                            delay((uint32_t)delayTime);
                        }
                        else
                        {
                            return false;
                        }
                    }
                    return true;
                }
            }
            else
            {
                return false;
            }
            return false;
        }
        else
        {
            delay(10);
        }
    }
    while (--tryCount);
    _nrfDebug =false;
//    uTime = second();
//    slotSec = calcNextSlotUnix(uTime, &nrfConfig);
//    slotSec = slotSec - second();
//    rtcTimerResetAlarm();
//    timerSetAlarm(slotSec,alarmTask);
    return false;
}

//void saveAddr(addr_t *addrPtr)
//{
//    SerialPrintln("NRF EEPROM Saving..");
//    unsigned int i = 0;
//    uint8_t *ptr = (uint8_t*) addrPtr;
//    uint8_t *writePtr = FRAM_NRF_DATA_SEND_ADDRESS;
//
//    SYSCFG0 = FRWPPW | PFWP;
//    for (i = 0; i < sizeof(addr_t); i++)
//    {
//        *writePtr++ = *ptr++;
//    }
//    SYSCFG0 = FRWPPW | PFWP | DFWP;
//
//    ptr = (uint8_t*) addrPtr;
//
//    nrfDebugBuffer(ptr, sizeof(addr_t));
//}
//
//void readAddr(addr_t *addrPtr)
//{
//    SerialPrintln("NRF EEPROM Reading..");
//    unsigned int i = 0;
//    uint8_t *ptr = (uint8_t*) addrPtr;
//    uint8_t *writePtr = FRAM_NRF_DATA_SEND_ADDRESS;
//    for (i = 0; i < sizeof(addr_t); i++)
//    {
//        *ptr++ = *writePtr++;
//    }
//    ptr = (uint8_t*) addrPtr;
//    nrfDebugBuffer(ptr, sizeof(addr_t));
//}


//void framRead(uint32_t addr, uint8_t *buf, uint16_t len)
//{
//  SerialPrintln("NRF FRAM Reading..");
//  unsigned int i = 0;
//  uint8_t *ptr = (uint8_t*) buf;
//  uint8_t *writePtr = (uint8_t*)addr;
//  for (i = 0; i < len; i++)
//  {
//      *ptr++ = *writePtr++;
//  }
////  ptr = (uint8_t*) addrPtr;
////  nrfDebugBuffer(ptr, sizeof(addr_t));
//}
//
//void framUpdate(uint32_t addr, uint8_t *buf, uint16_t len)
//{
//
//  SerialPrintln("NRF FRAM Saving..");
//  unsigned int i = 0;
//  uint8_t *ptr = buf;
//  uint8_t *writePtr = (uint8_t*)addr;
//
//  SYSCFG0 = FRWPPW | PFWP;
//  for (i = 0; i < len; i++)
//  {
//      *writePtr++ = *ptr++;
//  }
//  SYSCFG0 = FRWPPW | PFWP | DFWP;
//
////  ptr = (uint8_t*) addrPtr;
//
//}

// switch (clientState)
// {
// case CLIENT_READY:
//   clientState = CLIENT_IS_CONNECTED;
//   break;
// case CLIENT_IS_CONNECTED:
//   if (isBsConnected())
//   {
//     //            rtsync(); //sync time
//     nrfTxReady();
//     xferReady();
//     clientState = CLIENT_SEND;
//   }
//   else
//   {
//     clientState = CLIENT_CONNECT;
//   }
//   break;
// case CLIENT_SEND:
//   _nrfSendOk = xferSendLoopV3();
//   if (_nrfSendOk)
//   {
//     Serial.println(F("Done and wait"));
//     clientState = CLIENT_READY;
//     return true;
//   }
//   else
//   {
//     Serial.println(F("Failed and wait"));
//     clientState = CLIENT_READY;
//     return false;
//     //runState = RUN_CHK_BS_CONN;
//   }
//   break;
// default:
//   clientState = CLIENT_READY;
//   break;
// }
