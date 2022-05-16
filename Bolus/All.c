#include "All.h"
#include "utility.h"
#include <string.h>
 #define SYNC_PING_DELAY_MS 5000

uint32_t lastUnixTimeConnected;


void timePrint();
void useBackupTime();


bool isHardwareOk()
{
  SerialPrintln("<--Hardware Status-->");

  bool allOk = true;
  bool nrfOk = nrfIsRunning();
  SerialPrint("NRF :"); SerialPrintlnU8(nrfOk);
  allOk = allOk && nrfOk;

  bool flashOk = true;
  SerialPrint("Flash :"); SerialPrintlnU8(flashOk);
  allOk = allOk && flashOk;

  bool railVolt = true;
  SerialPrint("Logic Power: "); SerialPrintlnU8(railVolt);
  allOk = allOk && railVolt;

  SerialPrint("Hardware OK: "); SerialPrintlnU8(allOk);

  SerialPrintln("<------------------->");


  return allOk;
}

void alarmTask()
{
    SerialPrintln("->Task Called");
    _radioStartXfer = true;
}


void routineTask()
{
    static uint32_t _prevMillis;

    timePrint();
    uint32_t currentMillis = millis();
    if (currentMillis - _prevMillis >= 5000)
    {
//        SerialPrint("Time: ");
//        SerialPrintU32(second());
        SerialPrint("Millis: ");
        SerialPrintlnU32(millis());
        nrfWhichMode();
//        SerialPrint("Config : ");
//        SerialPrintlnU8(read_register(RF24_CONFIG));
//
        _prevMillis = currentMillis;
	}
}


void timePrint()
{
    static uint32_t lastSec;
    uint32_t nowSec = second();
    if(nowSec !=lastSec)
    {
        SerialPrintlnU32(second());
        lastSec = nowSec;
    }
}

/**********************New API*********************/

void mcuBasicSetup()
{
    //Set Radio Cs to low
    nrfSetPin(&P3OUT, NRF_CE_PIN, &P2OUT, NRF_CSN_PIN);
    //set flash cs to low
    flashPinSetup();  
    //set acc cs low   
    accCsLow();             
    //set serial and spi
    SerialBegin(SERIAL_SPEED);
    spi_begin(SPI_SPEED);
    //begin rtc timer 
    rtcTimerBegin();
}

void setupBasic()
{

    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer
    mspClockSet();
    PM5CTL0 &= ~LOCKLPM5;     // Disable the GPIO power-on default high-impedance mode
    _disable_interrupt();
    ds18b20_init_port();
}


bool syncDevice()
{

    lastUnixTimeConnected = readUnixTime();
#if defined(SHOW_DEBUG)
    SerialPrint("last Saved Unix Time: ");SerialPrintlnU32(lastUnixTimeConnected);
#endif
    int8_t trycount = 2;
    uint32_t uTime;
    do
    {
        uTime = nrfPing();
#if defined(SHOW_DEBUG)
        SerialPrint("NTP Time : ");SerialPrintlnU32(uTime);
#endif
        if(uTime)
        {
            setSecond(uTime);
#if defined(SHOW_DEBUG)
            SerialPrintln("RTC Time Set done");
#endif
            break;
        }
        else
        {
            delay(SYNC_PING_DELAY_MS);
        }
    }while(--trycount);
    bool conOk = nrfTxConfigHandler(DEVICE_ID, &nrfConfig, FRAM_NRF_DATA_SEND_ADDRESS, framRead, framUpdate);
    if(conOk)
    {
       uint32_t slotSec = calcNextSlotUnix(second(), &nrfConfig);

//       slotSec = slotSec - second();
//       rtcTimerResetAlarm();

//       timerSetAlarm(slotSec,alarmTask);
//       timerSetAlarm(slotSec,alarmTask);
       if(lastUnixTimeConnected > second())
       {
           useBackupTime();
       }else
       {
           rtcSetAlarm(slotSec,nrfConfig.momentDuration,alarmTask);
       }

    }else
    {
        useBackupTime();
//        uint32_t backUpTime = lastUnixTimeConnected;
//        SerialPrint("last Saved Unix Time: ");SerialPrintlnU32(lastUnixTimeConnected);
//        if(backUpTime > second())
//        {
//            setSecond(backUpTime);
//            backUpTime = calcNextSlotUnix(second(), &nrfConfig);
//            rtcSetAlarm(backUpTime,nrfConfig.momentDuration,alarmTask);
//        }
    }

    return true;
}


void useBackupTime()
{
    if(lastUnixTimeConnected > second())
    {
#if defined(SHOW_DEBUG)
        SerialPrint("last Unix Time: ");SerialPrintlnU32(lastUnixTimeConnected);
#endif
        setSecond(lastUnixTimeConnected);
        uint32_t backUpTime = calcNextSlotUnix(second(), &nrfConfig);
        rtcSetAlarm(backUpTime,nrfConfig.momentDuration,alarmTask);
    }
}

void printRunLog()
{
    sensorPrintLog();
    memqPrintLog(&memq);
}



// int main(void)
// {
// 	setupAll();
// 	mainState = CHECK_HARDWARE;


// 	isHardwareOk();
// 	startDevice();
// 	syncDevice();
// 	rfConfig();

// 	for (;;)
// 	{
// 		routineTask();

// 		switch (mainState)
// 		{
// 		case CHECK_HARDWARE:
// 		    if (isHardwareOk())
// 		    {
// //		       SerialPrintlnF(P("<==Hardware Ok==>"));
// 		       mainState = START_DEVICE;
// 		    }
// 		    else
// 		    {
// //		       SerialPrintlnF(P("<==Hardware Fault==>"));
// 		       delay(10000); //long delay restarts device due to watchdog
// 		    }
// 			break;
// 		case START_DEVICE:
// 			// SerialPrintln("|m_STATE: START_DEVICE|");
// 			startDevice();
// 			mainState = SYNC_TIME;
// 			//	        delay(2000);
// 			break;
// 		case SYNC_TIME:
// 			// SerialPrintln("|m_STATE: SYNCHRONIZE|");
// 			if (syncDevice())
// 			{
// //				nrfTxAddrHandler(framRead, framUpdate);//read addr from memory
// 				mainState = SYNC_RADIO;
// 				//	          nrfStandby1();
// 				SerialPrintln("|Going to SYNC_RF|");
// 				nrfPowerDown();
// //				nrfStandby1();
// 			}
// 			break;
// 		case SYNC_RADIO:
// 		    if(rfConfig())
//             {
// 		        mainState = DEVICE_RUN;
// 		        SerialPrintln("|Going to DEVICE_RUN|");
//             }
// 		    break;
// 		case DEVICE_RUN:
// 			// SerialPrintln("|m_STATE: DEVICE_RUN|");
// 			memq ->saveMemQPtr(memq);
// 			if(_radioStartXfer)
// 			{
// 			    nrfStandby1();
// 			    radioSendSM();
// 			    nrfPowerDown();
// 			    _radioStartXfer = false;
// 			}
// 			break;
// 		default:
// 			mainState = CHECK_HARDWARE;
// 			break;
// 		}
// 	}
// 	return 0;
// }


// void printMainState()
// {
// 	if (mainState != prevMainState)
// 	{
// 		switch (mainState)
// 		{
// 		case CHECK_HARDWARE:
// 			SerialPrintln("|m_STATE: CHECK_HARDWARE|");
// 			break;
// 		case START_DEVICE:
// 			SerialPrintln("|m_STATE: START_DEVICE|");
// 			break;
// 		case SYNC_TIME:
// 			SerialPrintln("|Going to SYNC_TIME|");
// 			break;
// 		case SYNC_RADIO:
// 		    SerialPrintln("|Going to SYNC_RADIO|");
// 		case DEVICE_RUN:
// 			SerialPrintln("|m_STATE: DEVICE_RUN|");
// 			break;
// 		case STOP:
// 			SerialPrintln("|m_STATE: STOP|");
// 			break;
// 		}
// 		prevMainState = mainState;
// 	}
// }
