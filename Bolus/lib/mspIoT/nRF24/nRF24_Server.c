/*
   nrf24_Server.c
   Created on:  Unknown
   Author: Shuvangkar Chandra Das & Shakil Anwar
   Description : nrf upper layer api for handling client quaries 
*/

#include"nRF24_Server.h"


uint8_t *nrfReadQuery(uint8_t *ptr, uint8_t len);
void PongHandler(query_t *query);

uint8_t _freeSlotId;
// uint8_t *pipe0AddrPtr;


//This function Configure query server and return true if successful 
bool nrfQueryBeginServer(volatile struct qryObj_t *qryObj)
{
    bool isOk;
    // pipe0AddrPtr = qryObj -> addr;
    

    isOk = (qryObj -> pipe >= 0 && qryObj -> pipe < 6) && 
            (qryObj -> activePingAddr != NULL) &&
            (_seconds != NULL) &&
            (_Millis != NULL);

    isOk =  isOk && 
        (qryObj -> bufPtr != NULL) && 
        (qryObj -> len > 0 && qryObj -> len <= 32) &&
        (qryObj -> pipeAddr != NULL) &&
        (qryObj -> resolver !=  NULL);

    nrfSetPipe(qryObj -> pipe, qryObj -> activePingAddr, true);
    

    if(isOk == false)
    {
        SerialPrintF(P("NRF QRY Begin Failed"));
    }
    return isOk;
}

//Set the address mode for server. decide server will  send response in which addresses 
void nrfTxSetModeServer(enum addrMode_t addrMode)
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
        nrfQryObj.activePingAddr = nrfQryObj.pipeAddr->pipe0Addr;
        break;
    case BS_PING:
        nrfQryObj.activePingAddr = nrfQryObj.pipeAddr->pipe1Addr;
        break;
    case BS_DATA:
        //concatenate pipe 2 addr
        // setBsDataMode(conPt);
        break;
    default:
        break;
    }
}

//End a particular query from server side.
//call this function in txIrq and mxRtIrq
void nrfQryServerEnd(query_t *qry)
{
    
    nrfTxSetModeServer(COMMON_PING);
    // SerialPrintF(P("Addr : ")); nrfPrintBuffer(nrfQryObj.addr, 5);

    nrfQryObj.pipe = 0;
    nrfSetPipe(nrfQryObj.pipe, nrfQryObj.activePingAddr, true);

    nrfRxStart();
    if(_nrfIrqState == NRF_SUCCESS)
    {
        SerialPrintlnF(P("<Qry Rcvd>"));
        if(qry -> opcode == NRF_CONFIG_OPCODE)
        {
           bool ok = tdmConfirmSlot(_freeSlotId);
        }
    }
    else
    {
        SerialPrintlnF(P("<Qry Failed>"));
    }
}


//This function handles all the quaries from the client. 
void nrfQueryHandler(query_t *query)
{
    // if (_nrfDebug)
    // {
    //     SerialPrintF(P("Qry type :")); SerialPrintU8(query.type);
    //     SerialPrintF(P(" | Qry opcode :")); SerialPrintlnU8(query.opcode);
    // }
    if(nrfQryObj.pipe > 0)
    {
        nrfTxSetModeServer(BS_PING);
        SerialPrintlnF(P("[NRF_QRY_SERVER]..Ping at BS Ping"));
    }
    else
    {
        nrfTxSetModeServer(COMMON_PING);
        SerialPrintlnF(P("[NRF_QRY_SERVER]..Ping at Common Ping"));
    }
    
    if(query -> type == PING_TYPE)
    {
        
     //Handle Internal query 
     switch(query -> opcode)
     {
        case PING_OPCODE:
        case PING2_OPCODE:
            PongHandler(query);
            break;
        case NRF_CONFIG_OPCODE:
            nrfServerConfigResolver(query);
            break;
     }
    }
    else
    {
        nrfQryObj.resolver(nrfQryObj.bufPtr);
        nrfToTxAndSend(nrfQryObj.activePingAddr, nrfQryObj.bufPtr, nrfQryObj.len);
    }
}


//This is a general function for responding ping request from server
void PongHandler(query_t *qry)
{  
    pong_t pong;
    //common pong data
    pong.type = PING_TYPE;
    pong.opcode = qry->opcode;
    pong.second = _seconds();
    pong.ms = _Millis();
    
    //specific pong data
    if(qry->opcode == PING2_OPCODE)
    {
        struct node_t *currentNode = (struct node_t*)tdmGetCurrentNode();
        // if(currentNode != NULL)
        // {
            SerialPrintF(P("Slot id : "));SerialPrintlnU8(currentNode -> slotNo);

            pong.isConfigChanged = false;
            
            if(currentNode -> deviceId == qry -> deviceId)
            {
            	pong.isBsFree = true;
	            pong.isMySlot = true;
	            
            }
            else
            {
            	pong.isBsFree = false;
            	pong.isMySlot = false;
            	if(tdmIsRegistered2(qry -> deviceId, qry-> slotId) == 255)
            	{
            		pong.isConfigChanged = true;
            	}
            }
            
        // }
        // else
        // {
        //     pong.isBsFree = false;
        //     pong.isMySlot = false;
        //     pong.isConfigChanged = false;
        //     SerialPrintlnF(P("Slot not synced"));
        // }
        
    }
    pong.checksum = checksum(&pong, sizeof(pong_t)-1);
    // pong.checksum = 10;
    nrfToTxAndSend(nrfQryObj.activePingAddr, (const uint8_t *)&pong, sizeof(pong_t));

}

//client requests configuration at the starting. This function resolves client config query
void nrfServerConfigResolver(query_t *qry)
{ 
    SerialPrintlnF(P("Resolving Config"));
    nrfNodeConfig_t config;
    // config.deviceId = qry -> deviceId;
    config.type = qry -> type;
    config.opcode = qry -> opcode;
    
    // uint8_t *addrPtr =  nrfQryObj.generateAddr(); //server generates adddress 
    memcpy(config.node, nrfQryObj.pipeAddr->pipe1Addr,sizeof(config.node));
    config.dataPipeLsbByte = nrfQryObj.pipeAddr->pipe2Addr;

    struct tdmMeta_t *tdmMeta = tdmGetMetaData();
    config.momentDuration = tdmMeta -> momentDuration;
    config.perNodeInterval = tdmMeta -> perNodeInterval;
    config.slotId =  tdmGetFreeSlot(qry -> deviceId);
    _freeSlotId = config.slotId;

    config.checksum = checksum(&config, sizeof(nrfNodeConfig_t)-1);

    nrfToTxAndSend(nrfQryObj.activePingAddr, (const uint8_t *)&config, sizeof(nrfNodeConfig_t));
    nrfPrintBuffer((uint8_t*)&config, sizeof(nrfNodeConfig_t));
}


