#include <msp430.h>
#include "All.h"


bool deviceSetup();
bool deviceActivate();
bool deviceRun();
bool devicePwrCtrl();
void deviceStatePrint();
void activationPrint();

enum deviceState_t
{
	DEVICE_SETUP,
	DEVICE_PRE_REG,
	DEVICE_ACTIVATION,
	DEVICE_NORMAL_RUN,
	DEVICE_POWER_CONTROL,
};

enum activationState_t 
{
	ACTIVATION_BEGIN,
	ACTIVATION_SYNC,
	ACTIVATION_END,
	ACTIVATION_WAIT
};

enum deviceState_t _deviceState;
enum activationState_t _activationState;

uint8_t rfSyncFail,rfBeginRetry;
uint16_t rfActivationFailCount;

int main(void)
{
    rfSyncFail = 0; rfActivationFailCount=0; rfBeginRetry = 0;
	setupBasic();
	_deviceState = DEVICE_SETUP;
	while(1)
	{
		switch(_deviceState)
		{
			case DEVICE_SETUP:
				deviceSetup();
				SerialPrintln("---->Device Setup done");
				_deviceState = DEVICE_ACTIVATION;
				break;
			case DEVICE_PRE_REG:
			break;
			case DEVICE_ACTIVATION:
				deviceActivate();
				break;
			case DEVICE_NORMAL_RUN:
				deviceRun();
//				__bis_SR_register(LPM4+GIE);
				break;
			case DEVICE_POWER_CONTROL:
				devicePwrCtrl();
				break;
			default:
			    break;
		}
//		deviceStatePrint();
//		routineTask();
	}
	return 0;
}







/********************************New API***************************/


bool deviceSetup()
{
    //Gpio & Communicatoin, timers etc affected
    mcuBasicSetup();
	//Radio Low  power mode
	radioSetLowPower(); //implement in radio
	//flash low power
	flashPowerDown();
	//Sensor Low power mode
#if defined(DEVICE_HAS_LIS3DH)
	acc_shutDown(); //implement in sensor.c
#endif
	//print debug for devicesetup
	nrfWhichMode();
#if defined(SHOW_DEBUG)
	SerialPrintln("<=======Setup Done==========>");
#endif
	SerialPrintln("pS Bolus v0.2.2");

	return true;
}

bool deviceActivate()
{
	switch(_activationState)
	{
		case ACTIVATION_BEGIN:
			//Begin Schema and others
			schemaBegin();
			// 1. Activate Sensor(Accelerometer)
#if defined(DEVICE_HAS_LIS3DH)
			sensorsBegin();
#endif
#if defined(DEVICE_HAS_TMP117)
			tmp117_init();
#endif
			// 2. Activate Radio
			radioBegin();
#if defined(FACTORY_RESET)
			nrfTxConfigReset(&nrfConfig, FRAM_NRF_DATA_SEND_ADDRESS, framUpdate);
#endif
			//flash and momory begin
			memoryBegin();
			//Start device 
			radioStart();
			//start timer
			rtcTimerStart();
			//Enable global interrupt
			_enable_interrupt(); // enable global interrupt
			_activationState = ACTIVATION_SYNC;
		break;
		case ACTIVATION_SYNC:
			//Time Sync  & Config and TDM sync
			syncDevice();
			_activationState = ACTIVATION_END;
		break;
		case ACTIVATION_END:
			//radio low power mode
			radioSetLowPower(); 
			//start sensing data
#if defined(DEVICE_HAS_LIS3DH)
			sensorStart();
#endif
			_activationState = ACTIVATION_WAIT;
            _deviceState = DEVICE_NORMAL_RUN;
		break;
		case ACTIVATION_WAIT:
		break;
		default:
			_activationState = ACTIVATION_BEGIN;
		break;
	}
//	activationPrint();
	return true;
}

bool deviceRun()
{
    // 1.Data loggin 
	// 2 Data Send 
//    memqSaveMemPtr(&memq);
//    float temp = ds18b20_get_temp();
//    SerialPrint("Device Temperature: ");
//    SerialPrintlnFloat(temp,2);

//    uint32_t *tempPtr;
//    uint64_t tempRom= readRom();
//    tempPtr = &tempRom;


//    SerialPrintU32(tempPtr[0]);
//    SerialPrint(" ");
//    SerialPrintlnU32(tempPtr[1]);
    if(_radioStartXfer)
    {
        readOutAccFifo();
        if((sensorStatus != SAVING_DATA) & (sensorStatus != READING_DATA))
        {
            nrfStandby1();
            delay(5);
            sensorStatus = SENDING_DATA;
            if(radioSendSM()<1)
            {
                rfSyncFail++;
            }
            else
            {
                rfSyncFail = 0;
            }
    //        radioSendSM();
#if defined(SHOW_DEBUG)
            SerialPrintln("<=====deviceRun End=====>");
#endif
            nrfPowerDown();
        }
        _radioStartXfer = false;
    }
    if(abs(second() - lastAccReadTime)>60)
    {
        sensorsBegin();
        sensorStart();
    }
    sensorStatus = IDLE;

#if defined(DEVICE_HAS_TEMP_SENSOR)
    readTemp();
#endif

//    printRunLog();
    if(_radioStartXfer){
        return true;
    }else{
        _deviceState = DEVICE_POWER_CONTROL;
        return false;
    }
}

bool devicePwrCtrl()
{
    static enum deviceState_t prevState = NULL;
    //  1.Alarm Set
	// 2. Sleep Control
    if(_radioStartXfer)
    {
        SerialPrintln("<RUN>");
        _deviceState = DEVICE_NORMAL_RUN;
        prevState = _deviceState;
    }else
    {
        if(prevState != _deviceState)
        {
            readOutAccFifo();
            flashPowerDown();
            SerialPrintln("<Sleep>");
            __low_power_mode_4();
//            delay(200);
            if(rfSyncFail > RF_ACTIVATION_RETRY)
            {
//                _activationState = ACTIVATION_SYNC;
//                _deviceState = DEVICE_ACTIVATION;
//                nrfTxSetModeClient(COMMON_PING,&nrfConfig);
#if defined(SHOW_DEBUG)
                SerialPrintln("Fetching new NRF Config");
#endif
                _radioStartXfer = false;
                nrfTxConfigHandler(DEVICE_ID, &nrfConfig, FRAM_NRF_DATA_SEND_ADDRESS, framRead, framUpdate);
                nrfPowerDown();
                rfBeginRetry++;
                rfSyncFail = 0;
                rfActivationFailCount++;
            }
            else
            {
                _deviceState = DEVICE_NORMAL_RUN;
            }

            if(rfBeginRetry > RF_BEGIN_RETRY)
            {
                nrfReset();
                radioBegin();
                radioStart();
                nrfTxSetModeClient(BS_PING, &nrfConfig);
                rfBeginRetry = 0;
            }

            prevState = _deviceState;

//            delay(50);
        }
    }
    return true;
}





void deviceStatePrint()
{
	static enum deviceState_t _preDevState;
	if(_deviceState != _preDevState)
	{
		SerialPrint("<--------_deviceState : ");
		switch(_deviceState)
		{
			case DEVICE_SETUP:
			SerialPrint("DEVICE_SETUP");
			break;
			case DEVICE_PRE_REG:
			SerialPrint("DEVICE_PRE_REG");
			break;
			case DEVICE_ACTIVATION:
			SerialPrint("DEVICE_ACTIVATION");
			break;
			case DEVICE_NORMAL_RUN:
			SerialPrint("DEVICE_NORMAL_RUN");
			break;
			case DEVICE_POWER_CONTROL:
			SerialPrint("DEVICE_POWER_CONTROL");
			break;
			default:
				SerialPrint("default");
			break;
		}
		SerialPrintln("-------->");
		_preDevState = _deviceState;
	}
}

void activationPrint()
{
	static enum activationState_t _preActState;
	if(_activationState != _preActState)
	{
		SerialPrint("<----_activationState : ");
		switch(_activationState)
		{
			case ACTIVATION_BEGIN:
			SerialPrint("ACTIVATION_BEGIN");
			break;
			case ACTIVATION_SYNC:
			SerialPrint("ACTIVATION_SYNC");
			break;
			case ACTIVATION_END:
			SerialPrint("ACTIVATION_END");
			break;
			case ACTIVATION_WAIT:
			SerialPrint("ACTIVATION_WAIT");
			break;
			default:
				SerialPrint("activation default");
			break;
		}
		SerialPrintln("---->");
		_preActState = _activationState;
	}
}
/**************New State **********************


Setup State
	1. Gpio Pin Setup 
	2. Communicatoin Protocol Setup 
	3. Sensor Low power mode
	6. Radio Low mode
	7. mcu low powe+ gpio in output mode(except hall sensor interrupt)
PRE_REGISTRATION Task 
	1. Magnet   Decoding 
	2. if successful store registration info in memory
	3. Register with base station. Then switch to post registration state. 
	4. handle After startup magneting event 
DEVICE_ACTIVATION,
	1. Activate Sensor(Accelerometer)
	2. Activate Radio
	3. Time Sync 
	4. Config and TDM sync 
Normal Running Modes 
	2. Data login
	3. Data Send 
Sleep Control Mode
	1. Alarm Set
	2. Sleep Control  
*/
