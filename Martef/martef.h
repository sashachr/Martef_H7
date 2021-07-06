// MARTEF.H
// COPYRIGHT 2012 Sasha Chrichov
#pragma once

struct TimerStruct {
	uint32_t Tick, Milliseconds, Seconds;
	uint32_t Period;
	uint16_t millisecondTick, secondTick;
    uint16_t initialDelayCounter;
    float Late, XLate, Use, XUse, percentFactor;
};

extern struct TimerStruct Timer;

void MartefInit();
void MartefTick();
