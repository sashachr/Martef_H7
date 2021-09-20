#include "chip.h"

#include "global.h"
#include "thread.h"
#include "timer.h"
#include "martef.h"
#include "systick.h"

uint32_t SysTickInit() {
	if (SysTick_Config(Time.Period)) {
		return 1;
	}
	return 0;
}

//extern "C" __thumb void SysTick_Handler();
extern "C" __attribute__((naked,noreturn,optimize("-O3"))) void SysTick_Handler() {
	SwitchToDefaultStack;
    MartefTick();
	SwitchToNextThreadStack;
}

