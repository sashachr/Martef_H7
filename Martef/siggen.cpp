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
	switch (Type) {
		case SGN_SQUARE:
			if (count == 0) Sgn = Max;
			if (++count >= period) { count = 0;  }
			if (count == (period >> 1)) Sgn = Min;
			break;
		case SGN_RAMP: {
			if (++count >= period) { count = 0; down = !down; }
			float a = ((float)count)/period;
			Sgn = down ? (1 - a) * Max + a * Min : (1 - a) * Min + a * Max;
			break;
		}
		case 3:
			if (++count >= period) { count = 0;  }
			Sgn = (count < Duty*period) ? Max : Min;
			break;
		case 4:
			if (++count >= period) { count = 0;  }
			Sgn = 0.5F*((Min+Max) + (Max-Min)*sinf((2*M_PI*count)/period));
			break;
		case 5:
			if (++count >= period) { count = 0;  }
			Sgn = 0.5F*((Min+Max) + (Max-Min)*cosf((2*M_PI*count)/period));
			break;
	}
}

int32_t SignalSetPeriod(uint16_t ind, uint16_t count, int32_t* buf) {
	for (uint16_t i = 0; i < count; i++, ind++, buf++) {
		if (ind < 2) Signals[ind].SetPeriod(*(float*)buf);
	}
}
