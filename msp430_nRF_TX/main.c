#include <msp430.h> 
#include <mspDriver.h>
#include <mspIoT.h>

/**
 * main.c
 */
void txIsr(void);
void rxIsr(void);
void maxRtIsr(void);

uint32_t prevMillis = 0;

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
    mspClockSet();

    timerBegin();
    timerMillisStart();
    timerRtcStart();

    SerialBegin(115200);
    SerialPrintln("=>Serial Setup Done");
//	nrfSetPin(&P3OUT, 7, &P3OUT, 6);
//	nrfBegin(SPEED_2MB, POWER_ZERO_DBM, 1000000);
    SerialPrintln("Setup Done");
    _enable_interrupt(); // enable global interrupt
    while (1)
    {
//        SerialPrintlnU32(second());

        if (millis() - prevMillis >= 1000)
        {
            SerialPrint("\r\nTime: ");
            SerialPrintU32(second());
            SerialPrint(" | Millis: ");
            SerialPrintU32(millis());
            prevMillis = millis();
        }
    }
    return 0;
}
