// MARTEF.CPP
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"

#include "global.h"
#include "thread.h"
#include "pins.h"
#include "systick.h"
#include "martef.h"

TimerStruct Timer;

void MartefInit() {
	Timer.Tick = Timer.Milliseconds = Timer.Seconds = 0;
	Timer.millisecondTick = Timer.secondTick = 0;
    Timer.initialDelayCounter = 100;        // 100 milliseconds
	/* Configure SysTick for 50 mksec. */
    Timer.Period = SYSCLK_RATE/(int)TICKS_IN_SECOND;
    Timer.percentFactor = 100.0F/Timer.Period;
	ThreadsInit();
	PinsInit();
	SysTickInit();
}

uint32_t Leds = 0x00000008;

void MartefTick() {
//    GPIOB->BSRRL = 0x0200;      // Set FAULT output
//    GPIOD->BSRRH = 0x1000;      // DIGITAL_OUT = 1
    Timer.Late = (Timer.Period - SysTick->VAL) * Timer.percentFactor;
    if (Timer.XLate < Timer.Late) Timer.XLate = Timer.Late;
    Timer.Tick++;
	if (++Timer.millisecondTick == (int)TICKS_IN_MILLISECOND) {
		Timer.millisecondTick = 0;
        Timer.Milliseconds++;
        if (Timer.initialDelayCounter) {
            if (--Timer.initialDelayCounter == 0) {
            }
        }
		if (++Timer.secondTick == 1000) {
			Timer.secondTick = 0;
            Timer.Seconds++;
			GPIOC->BSRR = Leds;
			Leds ^= 0x00080008;
		}
	}
//    Adc.Tick();
    if (!Timer.initialDelayCounter) {
//        Servo.Tick();
//        Dac.Tick();
//        LedStatus.Tick();
//        Scope.Tick();
//        CommunicationTick();
    }
    Timer.Use = (Timer.Period - SysTick->VAL) * Timer.percentFactor;
    if (Timer.XUse < Timer.Use) Timer.XUse = Timer.Use;
}
