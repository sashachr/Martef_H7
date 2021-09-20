// TIME.CPP
// COPYRIGHT 2015 Sasha Chrichov

#include "chip.h"

#include "global.h"
#include "thread.h"
#include "pins.h"
#include "martef.h"
#include "timer.h"

TimeStruct Time;

void TimeInit() {
	Time.Tick = Time.Milliseconds = Time.Seconds = 0;
	Time.inMillisecondCount = Time.inSecondCount = 0;
    Time.initialDelayCount = 100;        // 100 milliseconds
	/* Configure SysTick for 50 mksec. */
    Time.Period = SYSCLK_RATE/(int)TICKS_IN_SECOND;
    Time.percentFactor = 100.0F/Time.Period;
}

uint32_t Leds = 0x00000008;

void TimeTickStart() {
    Time.Late = (Time.Period - SysTick->VAL) * Time.percentFactor;
    if (Time.XLate < Time.Late) Time.XLate = Time.Late;
    Time.Tick++;
	if (++Time.inMillisecondCount == (int)TICKS_IN_MILLISECOND) {
		Time.inMillisecondCount = 0;
        Time.Milliseconds++;
        if (Time.initialDelayCount) Time.initialDelayCount--;
		if (++Time.inSecondCount == 1000) {
			Time.inSecondCount = 0;
            Time.Seconds++;
			GPIOC->BSRR = Leds;
			Leds ^= 0x00080008;
		}
	}
}

void TimeTickEnd() {
    Time.Use = (Time.Period - SysTick->VAL) * Time.percentFactor;
    if (Time.XUse < Time.Use) Time.XUse = Time.Use;
    GPIOD->BSRR = 0x00040000;           // Reset D2
}
