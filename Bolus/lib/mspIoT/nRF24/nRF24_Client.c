/*
   nrf24_Client.c
   Created on:  Unknown
   Author: Shuvangkar Chandra Das & Shakil Anwar
   Description : This provide api for write clinet firmware
*/

#include "nRF24_Client.h"

void setBsDataMode(nrfNodeConfig_t *conPt);
uint8_t *pipe0AddrPtr;

nrfNodeConfig_t *nrfTxGetConfig(uint16_t DeviceId, nrfNodeConfig_t *configPtr);
uint8_t *nrfReadQuery(uint8_t *ptr, uint8_t len);


//nrf query client begin function
bool nrfQueryBeginClient(volatile struct qryObj_t *qryObj)
{
    bool isOk;
    pipe0AddrPtr = qryObj -> activePingAddr;
    SerialPrintF(P("Addr : ")); nrfPrintBuffer(qryObj -> activePingAddr, 5);
    isOk =  (qryObj -> pipe >= 0 && qryObj -> pipe < 6) && 
            (qryObj -> activePingAddr != NULL) &&
            (_seconds != NULL) &&
            (_Millis != NULL);

    if(isOk == false)
    {
        SerialPrintF(P("NRF QRY Begin Failed"));
    }
    return isOk;
}

//This function handles sensor node configuration parameters
bool nrfTxConfigHandler(uint16_t DeviceId, nrfNodeConfig_t *conf,
                        uint32_t romAddr,nrfMemFun_t read,nrfMemFun_t save)
{
    read((uint32_t)romAddr,(uint8_t*)conf,sizeof(nrfNodeConfig_t));
    //match checksum to validate that data does erased in memory
    uint8_t checksumCalc = checksum(conf, sizeof(nrfNodeConfig_t)-1);
    bool isConfOk = (checksumCalc == conf->checksum) && 
                    (conf -> type == PING_TYPE) &&
                    (conf -> opcode == NRF_CONFIG_OPCODE);

    nrfNodeConfig_t *conf_t_ptr;
    SerialPrintlnF(P("Getting New Config"));
    nrfTxSetModeClient(COMMON_PING,conf);
    conf_t_ptr = nrfTxGetConfig(DeviceId,conf);

    if(conf_t_ptr !=NULL)
    {
        
        // SerialPrintlnF(P("[nRF24_Client]..Device saves new config"));
        uint8_t checksumCalc_t = checksum(conf_t_ptr, sizeof(nrfNodeConfig_t)-1);
        if(checksumCalc_t == checksumCalc)
        {
            SerialPrintlnF(P("[nRF24_Client]..Device uses old config"));
        }
        else
        {
            SerialPrintlnF(P("[nRF24_Client]..Device saves new config"));
            save((uint32_t)romAddr,(uint8_t*)conf_t_ptr,sizeof(nrfNodeConfig_t));
        }
        nrfTxSetModeClient(BS_PING,conf);
        return true;
    }
    else
    {
        //config qry failed
        if(isConfOk)
        {
        	SerialPrintlnF(P("[nRF24_Client]...Device uses old config"));
            nrfTxSetModeClient(BS_PING,conf);
        }else{
            SerialPrintlnF(P("[nRF24_Client]..Device has no config"));
        }
        return isConfOk;
    }

    // if(isConfOk == false)                           
    // {
    //     //Device has no config, or erased, get the new config
    //     if(_nrfDebug){ SerialPrintlnF(P("Getting New Config"));}
    //     conf = nrfTxGetConfig(DeviceId,conf);
    //     if(conf !=NULL)
    //     {
    //         nrfTxSetModeClient(BS_PING,conf);
    //         save((uint32_t)romAddr,(uint8_t*)conf,sizeof(nrfNodeConfig_t));
    //         return true;
    //     }
    //     else
    //     {
    //         //config qry failed
    //         return false;
    //     }
    // }
    // else
    // {
    //     if(_nrfDebug)
    //     { 
    //         SerialPrintlnF(P("Device has Config"));
    //         nrfPrintConfig(conf);
    //     }
    // }
    // return isConfOk;
}


//request server for configuration 
nrfNodeConfig_t *nrfTxGetConfig(uint16_t DeviceId, nrfNodeConfig_t *configPtr)
{
//   nrfNodeConfig_t addr;
  nrfStandby1();
  nrfTxStart();
  query_t query;
  query.type = PING_TYPE;
  query.opcode = NRF_CONFIG_OPCODE;
  query.deviceId = DeviceId;

  configPtr = nrfQuery(&query,configPtr,sizeof(nrfNodeConfig_t));
  if (configPtr != NULL)
  {
    nrfPrintConfig(configPtr);
  }
   return configPtr;
}


//Clear configuration
void nrfTxConfigReset(nrfNodeConfig_t *conf, uint16_t romAddr,nrfMemFun_t save)
{
    memset(conf,0,sizeof(nrfNodeConfig_t));
    save((uint32_t)romAddr,(uint8_t*)conf,sizeof(nrfNodeConfig_t));
}

//Set address mode for client operation
void nrfTxSetModeClient(enum addrMode_t addrMode,nrfNodeConfig_t *conPt)
{
    //  nrfTxBegin(conPtr -> node, true);
    // nrfStandby1();
    // nrfTxStart();
    // if(_nrfDebug)
    // {
    //     SerialPrintlnF(P("TX Ready"));
    // }     
    switch (addrMode)
    {
    case COMMON_PING:
        nrfQryObj.activePingAddr = pipe0AddrPtr;
        break;
    case BS_PING:
        nrfQryObj.activePingAddr = conPt->node;
        break;
    case BS_DATA:
        //concatenate pipe 2 addr
        setBsDataMode(conPt);
        break;
    default:
        break;
    }
}

//Set the data send mode in tx address
void setBsDataMode(nrfNodeConfig_t *conPt)
{
    uint8_t dataPipeAddr[5];
    strncpy((char *)dataPipeAddr,(char *)conPt -> node,5);
    dataPipeAddr[0]= conPt -> dataPipeLsbByte;
    // SerialPrintF(P("Data Send Addr : ")); nrfPrintBuffer(dataPipeAddr, 5);
    nrfTxBegin(dataPipeAddr, true);
    nrfStandby1();
    nrfTxStart();
    if(_nrfDebug)
    {
        SerialPrintlnF(P("TX Ready"));
    } 
}

//When client send query and server responde, this function reads the query response
uint8_t *nrfReadQuery(uint8_t *ptr, uint8_t len)
{
    switch (_nrfIrqState)
    {
    case NRF_SUCCESS:
        // nrfTxBegin(_mainTxAddr,true);//After query send change tx address to previous tx addr.

        nrfSetPipe(nrfQryObj.pipe, nrfQryObj.activePingAddr, true); //Change to rx mode
        nrfRxStart();
        #if defined(ARDUINO_ARCH_AVR)
        // if (_nrfDebug)
        // {
        //     SerialPrintlnF(P("NRF Query Accepted"));
        // }
        #endif
        _nrfIrqState = NRF_WAIT;
        break;
    case NRF_WAIT:
        // //serial_print("\r\n Q_WAIT");
        break;
    case NRF_RECEIVED:
        #if defined(ARDUINO_ARCH_AVR)
        // if (_nrfDebug)
        // {
        //     SerialPrintlnF(P("NRF Query Received"));
        // }
        #endif
        // SerialPrintF(P("Len : "));SerialPrintlnU8(nrfPayloadLen());
        nrfRead(ptr, len);
        return ptr;
        break;
    }
    return NULL;
}

//This is the query base function. This function send query to the server and return response
uint8_t *nrfQuery(query_t *qry,void *bufPtr, uint8_t len)
{
    if(_nrfDebug){SerialPrintlnF(P("Querying.."));}
    
    nrfToTxAndSend(nrfQryObj.activePingAddr,(uint8_t *)qry, sizeof(query_t));

    uint32_t currentMillis = _Millis();
    uint32_t prevMillis = currentMillis;
    uint8_t *queryPtr = NULL;
    do
    {
        queryPtr = nrfReadQuery((uint8_t*)bufPtr,len);
        if (queryPtr != NULL)
        {
            nrfPrintBuffer(queryPtr, len);
            len--;//pointing last byte
            uint8_t calcCheckSum = checksum(queryPtr,len);
            if(calcCheckSum == queryPtr[len])
            {
                if(_nrfDebug){SerialPrintlnF(P("<Qry Ok>"));} 
            }
            else
            {
                queryPtr = NULL;
                if(_nrfDebug)
                {
                    SerialPrintlnF(P("<Qry Mismatch>"));
                    // SerialPrintF(P("Expected : ")); SerialPrintU8(queryPtr[len]);
                    // SerialPrintF(P(" | Received : ")); SerialPrintlnU8(calcCheckSum);
                }
            }
            break;
        }      
        currentMillis = _Millis();
    } while ((currentMillis - prevMillis) < QUERY_TIMEOUT_MS);

    if(queryPtr == NULL)
    {
        if(_nrfDebug){SerialPrintlnF(P("<Qry Failed>"));}
    }
    return queryPtr;
}


pong_t *nrfping(query_t *qry,pong_t *ping)
{
  nrfStandby1();
  nrfTxStart();
  
  pong_t *pongPtr = (pong_t *)nrfQuery(qry,(void *)ping,sizeof(pong_t));
  if (pongPtr != NULL)
  {
//       printPing(pongPtr);
  }
  return pongPtr;
}

//fast ping function which returns unix time
uint32_t nrfPing()
{
    query_t query;
    pong_t pong;

    query.type = PING_TYPE;
    query.opcode = PING_OPCODE;
    pong_t *ponPtr = nrfping(&query,&pong);
    if(ponPtr != NULL)
    {
        return ponPtr -> second;
    }
    return 0;
}

uint32_t nrfPingSlot(uint16_t deviceId, uint8_t slotId, struct pong_t *pong)
{
    query_t query;
    // pong_t pong;

    query.type = PING_TYPE;
    query.opcode = PING2_OPCODE;
    query.deviceId = deviceId;
    query.slotId = slotId;

    pong_t *ponPtr = nrfping(&query,pong);
    if(ponPtr != NULL)
    {
         /*
        ==========================================
        isBsFree    |   isMyslot    |   okToSend
        ------------------------------------------
            0       |        0      |         0
            0       |        1      |         0           
            1       |        0      |         1
            1       |        1      |         1
        ==========================================
        */
#if defined(ARDUINO_ARCH_AVR)
       if(ponPtr ->isBsFree && ponPtr ->isMySlot)
       {
           return ponPtr -> second;
       }
       else
       {
       		return 1;
       }
    }
    return NULL;
#elif defined(__MSP430G2553__) || defined(__MSP430FR2433__)
       if(ponPtr ->isBsFree && ponPtr ->isMySlot)
       {
           return ponPtr -> second;
       }else
       {
           return 1;
       }
        // return (ponPtr ->isBsFree && ponPtr ->isMySlot);
    }
    return 0;
#endif
}
