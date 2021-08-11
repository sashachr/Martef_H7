// Copyright (c) Sasha (Alexander) Chrichoff. All rights reserved.
// Under no circumstances may you delete or modify this file header.
// See LICENSE.TXT file in the project root for full license information.  
//
#pragma once

#define SGN_NONE    0
#define SGN_SQUARE  1
#define SGN_RAMP    2
#define SGN_PULSE   3
#define SGN_SIN     4
#define SGN_COS     5
 

class SignalStruct {
public:
    uint8_t Type;
    float Period, Duty, Min, Max;
    float Sgn;
    SignalStruct(): Type(0), Min(-100), Max(100), Duty(0.5F) { SetPeriod(0.01F); }
    void Tick();
    void Reset() {count = 0; down = 0; }
    void SetPeriod(float p) {
        Period = p;
        period = (uint32_t)round(p * TICKS_IN_SECOND);
        rate = 1.0F/period;
        Reset();
    }
private:
    uint32_t period, count;
    float rate;
    uint8_t down;
};

extern SignalStruct Signals[2];

int32_t SignalSetPeriod(uint16_t ind, uint16_t count, int32_t* buf);

