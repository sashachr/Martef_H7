// MARTEF.H
// COPYRIGHT 2012 Sasha Chrichov
#pragma once

struct TimeStruct {
	uint32_t Tick, Milliseconds, Seconds;
	uint32_t Period;
	uint16_t inMillisecondCount, inSecondCount;
    uint16_t initialDelayCount;
    float Late, XLate, Use, XUse, percentFactor;
};

extern struct TimeStruct Time;

void TimeInit();
void TimeTickStart();
void TimeTickEnd();
