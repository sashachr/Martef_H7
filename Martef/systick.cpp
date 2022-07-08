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
uint32_t SysTickIntDisable() {
	SysTick->CTRL &= ~0x00000002;
	return 0;
}

//extern "C" __thumb void SysTick_Handler();
extern "C" __attribute__((naked,noreturn,optimize("-O3"))) void SysTick_Handler() {
	SwitchToDefaultStack;
GPIOF->BSRR = 0x00008000;                  // F15 = 1
    MartefTick();
GPIOF->BSRR = 0x80000000;                  // F15 = 0
	SwitchToNextThreadStack;
}

