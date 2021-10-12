// Copyright (c) Sasha (Alexander) Chrichoff. All rights reserved.
// Under no circumstances may you delete or modify this file header.
// See LICENSE.TXT file in the project root for full license information.  
//
#include "chip.h"
#include <math.h>

#include "global.h"
#include "siggen.h"

SignalStruct Signals[2];

void SignalStruct::Tick() {
	if (Type == 0) return;
	int type = Type;
	float from, to;
	if (type < 100) { // symmetrical
		from = -Max; to = Max;
	} else if (type < 200) { // zero-based
		from = 0; to = Max; type -= 100;
	} else { // min-max
		from = Min; to = Max; type -= 200;
	}
	if (++count >= period) count = 0; 
	switch (type) {
		case SGN_SQUARE:
			Sgn = (count < (period >> 1)) ? from : to;
			break;
		case SGN_RAMP: {
			int up = count < (period >> 1);
			float a = up ? (float)count/(period >> 1) : (float)(period - count)/(period - (period >> 1));
			Sgn = (1 - a) * from + a * to;
			break;
		}
		case SGN_PULSE:
			Sgn = (count < Duty*period) ? to : from;
			break;
		case SGN_SIN:
			Sgn = 0.5F*((from+to) + (to-from)*sinf((2*M_PI*count)/period));
			break;
		case SGN_COS:
			Sgn = 0.5F*((from+to) + (to-from)*cosf((2*M_PI*count)/period));
			break;
	}
}

int32_t SignalSetPeriod(uint16_t ind, uint16_t count, int32_t* buf) {
	for (uint16_t i = 0; i < count; i++, ind++, buf++) {
		if (ind < 2) Signals[ind].SetPeriod(*(float*)buf);
	}
	return 1;
}
