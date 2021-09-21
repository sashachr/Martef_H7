// MARTEF.H
// COPYRIGHT 2012 Sasha Chrichov
#pragma once

struct TimeStruct {
	uint32_t Tick, Milliseconds, Seconds;
	uint32_t Period;
	uint16_t inMillisecondCount, inSecondCount;
    uint16_t initialDelayCount;
    float Late, XLate, Use, XUse, percentFactor;
	float GetTime() { return Tick * SECONDS_IN_TICK; }
	void SetTime(float time) { Tick = (uint32_t)(time * TICKS_IN_SECOND); }
};

extern struct TimeStruct Time;

void TimeInit();
void TimeTickStart();
void TimeTickEnd();
