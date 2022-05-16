/*
Author : Shuvangkar Chandra Das
Contributor :  Shakil Anwar 
Description : In communication we need asynchronous data sending mechanimsm. 
This module provides this feature. This is a general purpose asynchronous module.
for data sending. So it can be used with communcation mechanism like sim800l,  wifi, rf etc. 
*/

#include "asyncXfer.h"

#if defined(ARDUINO_ARCH_AVR)
    #include <Arduino.h>
    #if defined(PROD_BUILD)
        #include "../arduinoCwrapper/Serial.h"
    #else
        #include "Serial.h"
    #endif
#elif defined(ARDUINO_ARCH_SAM)
    #include <Arduino.h>
#elif defined(__MSP430G2553__) || defined(__MSP430FR2433__)
    #include <msp430.h>
    #include "../../mspDriver/mspDriver.h"
#else
    #error "nRF24_DRIVER did not find chip architecture "
#endif


enum state_e
{
    READ_MEM,
    TO_JSON,
    SERVER_SEND,
    WAIT_ACK,
    SEND_SUCCESS,
    FAILED,
};


enum state_e sendState;

//Callback functions
read_t _read;
send_t _send;
ackWait_t _ackWait;
millis_t _millis;


bool _isReady;
uint8_t _maxTryCount;
bool _debug;


//Begin function for xfer, it takes the user defined function to perform sending mechanism. 
void xferBegin(read_t read, send_t send, ackWait_t ackWait, millis_t ms)
{
    _read = read;
    _send = send;
    _ackWait = ackWait;
    _millis = ms;

    _isReady = false;
    _maxTryCount = 1; //default try once
    _debug = true;    //default Serial debug on
    // SerialBegin(9600);
    // SerialPrintlnF(P("asyncXfer setup done"));
}

//This function enable or disable debug with maxtry limit. 
void xferConfig(uint8_t mxTry,bool debug)
{
    _maxTryCount = mxTry;
    _debug = debug;
}

//This function needs to call once before start sending mechanism "xferSendLoop"
//This just initiate the state machine 
void xferReady()
{
    _isReady = true;
    sendState = READ_MEM;
}

///The function send data in asynchronous way. So It can work parallel with main loop. 
//This is good for parallel operation with other function
bool xferSendLoop()
{
    static uint8_t *_ptr = NULL;
    static uint32_t _startMillis;
    static uint8_t  _totalTime;
    static int _ackCode;
    static uint8_t _tryCount;

    if (_isReady)
    {
        switch (sendState)
        {
        case READ_MEM:
            _ptr = _read();
            if (_ptr != NULL)
            {
                if(_debug) 
                {
                    _startMillis = _millis();
                    // SerialPrintlnF(P("Xfer : READ MEM : OK"));
                }
                sendState = SERVER_SEND;
                _tryCount = 0; //resets try count
            }
            break;
        case SERVER_SEND:
            // if(_debug) {SerialPrintlnF(P("Xfer : SERVER_SEND"));}
            _send(_ptr);
            _tryCount++;
            sendState = WAIT_ACK;
            break;
        case WAIT_ACK:
            // if(_debug) {SerialPrintlnF(P("Xfer : WAIT_ACK"));}
            _ackCode = _ackWait();
            if (_ackCode == 200)
            {
                sendState = SEND_SUCCESS;
            }
            else
            {
                sendState = FAILED;
            }

            break;
        case SEND_SUCCESS:
            if(_debug) 
            {
                _totalTime = _millis() -  _startMillis;
                SerialPrintF(P("XFER->OK|Time:"));
                SerialPrintlnU32(_totalTime);
            }
            _isReady = true;
            sendState = READ_MEM;
            return _isReady;
            break;
        case FAILED:
            if(_debug) {SerialPrintF(P("XFER->NOK"));}
            if(_tryCount < _maxTryCount)
            {
                if(_debug) {SerialPrintlnF(P("|Resending"));}
                sendState = SERVER_SEND;
            }
            else
            {
                if(_debug) {SerialPrintlnF(P("|Exiting"));}
                _isReady = false;
            }
            break;
        default:
            sendState = READ_MEM;
            break;
        }
    }
    return _isReady;
}


bool xferSendLoopV2()
{
    static uint8_t *_ptr = NULL;
    static uint32_t _startMillis;
    static uint8_t  _totalTime;
    static int _ackCode;
    static uint8_t _tryCount;

	do
	{
        switch (sendState)
        {
            case READ_MEM:
                _ptr = _read();
                if (_ptr != NULL)
                {
                    if(_debug) 
                    {
                        _startMillis = _millis();
                        SerialPrintlnF(P("Xfer : READ MEM : OK"));
                    }
                    sendState = SERVER_SEND;
                    _tryCount = 0; //resets try count
                }
            break;
            case SERVER_SEND:
            if(_debug) {SerialPrintlnF(P("Xfer : SERVER_SEND"));}

            _send(_ptr);
            _tryCount++;
            sendState = WAIT_ACK;
            break;

            case WAIT_ACK:
            if(_debug) {SerialPrintlnF(P("Xfer : WAIT_ACK"));}
            _ackCode = _ackWait();
            if (_ackCode == 200)
            {

                sendState = SEND_SUCCESS;
                // _isReady = true;
            }
            else
            {
                sendState = FAILED;
                // _isReady = false;
            }

            break;
            case SEND_SUCCESS:
            if(_debug) 
            {
                SerialPrintF(P("Xfer : SEND_SUCCESS | Xfer Time : "));
                _totalTime = _millis() -  _startMillis;
                SerialPrintlnU32(_totalTime);
            }

            _isReady = true;
            sendState = READ_MEM;
            return _isReady;
            break;
            case FAILED:
            if(_tryCount < _maxTryCount)
            {
                if(_debug) {SerialPrintlnF(P("Xfer : FAILED | Resending"));}
                sendState = SERVER_SEND;
            }
            else
            {
                if(_debug) {SerialPrintlnF(P("Xfer : FAILED | Exiting xferSendLoop"));}
                _isReady = false;
            }
            
            break;

    	}

	}while(_ptr != NULL && _isReady);
    return _isReady;
}


//This function block in a while loop and send all data and breaks the loop. 
//This is good when data need to send fast and ona single try. 
bool xferSendLoopV3()
{
     static uint8_t *_ptr = NULL;
    static uint32_t _startMillis;
    static uint8_t  _totalTime;
    static int _ackCode;
//    static uint8_t _tryCount;

    uint16_t totalXfer = 0;
    if(_debug) 
    {
       _startMillis = _millis();
    }

    do
    {
        _ptr = _read();
        if (_ptr != NULL)
        {
            // if(_debug) 
            // {
            //     _startMillis = _millis();
            // }
            _send(_ptr);
            _ackCode = _ackWait();
            if(_ackCode == 200)
            {
                _isReady = true;
                totalXfer++;
                // if(_debug)
                // {
                //      _totalTime = _millis() -  _startMillis;                   
                //      SerialPrintF(P("XFER->OK|Time:"));
               	// 	 SerialPrintlnU32(_totalTime);
                // } 
            }
            else
            {
                if(_debug) {SerialPrintF(P("XFER->NOK"));}
                _isReady = false;
            }
        }

    }while(_ptr != NULL && _isReady);
    
    if(totalXfer>0)
    {
    	_totalTime = _millis() -  _startMillis;   
        SerialPrintF(P("XFER->TOTAL_PKT:"));
        SerialPrintU16(totalXfer);
        SerialPrintF(P("|Time:"));SerialPrintlnU32(_totalTime);
    }
    return _isReady;
}

