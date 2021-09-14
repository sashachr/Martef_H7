// Servo.cpp
// Servo loop
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"
#include <math.h>
#include <new>

#include "global.h"
#include "adc.h"
#include "pwm.h"
#include "io.h"
#include "siggen.h"
#include "motion.h"
#include "servo.h"

ServoStruct Servo[NAX];

void ServoStruct::Init(uint8_t index) {
    RState = FState = 0;
    Index = index;
    Motion = NewMotion(index);
	Pwm.Init();
//	Pwm.SetFrequency(80000);
    float bq[] = {100.0F, 0.7F};    // Bandwidth 700 Hz, Damping 0.7
    // BqDc.Config(BQ_LPF, bq);
    // BqAc.Config(BQ_LPF, bq);
    InitialCounter = 10000;         // 0.5 sec per Vlad's request
}

void ServoStruct::Enable(uint8_t en) {
    if (en) {
        if (Error) SetError(0);
    } 
    RState |= en;
}

uint16_t ServoStruct::GetError(uint32_t safety) {
    if (safety & SB_MOTORLOST) return FLT_MOTORLOST; 
    if (safety & SB_EMERGENCY) return FLT_EMERGENCY;
    if (safety & SB_UNKNOWNMOTOR) return FLT_UNKNOWNMOTOR;
    if (safety & 0x00010000) return FLT_OVERVOLTAGE;
    if (safety & 0x00200000) return FLT_ACCURRENT;
    if (safety & 0x00400000) return FLT_DCCURRENT;
    if (safety & 0x021E0000) return FLT_VOLTAGE;
    return 0;
}

void ServoStruct::SetError(uint16_t error) {
    Error = error;
    if (error != 0) {
//        Io.Io |= IO_FAULT;
    } else {
//        Io.Io &= ~IO_FAULT;
    }
}

void ServoStruct::Tick() {
    if (InitialCounter) InitialCounter--;
    Io.UpdateInputs();
    uint32_t iochange = io ^ Io.Io;
	if (iochange) {
	}
    SafetyRaw = 0; // SafetyBits();
    Safety = SafetyRaw & ~SafetyMask;
    if (Safety) {
        if (Error == 0) SetError(GetError(Safety));
    }
    if (Error != 0) {
        RState &= ~SM_ENABLE;
    }
//     uint32_t modechange =  mode ^ Mode;
//     if (modechange) {
//         mode = Mode;
//         if (modechange & SM_ENABLE) Enable(Mode & SM_ENABLE);
//     }
//     if ((modechange & SM_DC) && !InTransition) {
// 		if (Enabled()) {
//             Pwm.Enable = (Mode & SM_DC) == 0;
//             SetMinMax();
//         }
//     }
// 	if ((modechange & SM_ENABLE) && !InTransition) {
//         Pwm.Enable = 0;
// 		if (Enabled()) {
//             SetMinMax();
//             Pwm.Enable = (Mode & SM_DC) == 0;
//             Ploop.Reset(); Vloop.Reset();
//         } else {
//             POut = VOut = Pwm.In = 0;
//         }
// 	}
// 	if ((modechange & SM_LINEAR) && !InTransition) {
// 		Pwm.Mode = LinearMode();
//         if (Enabled()) SetMinMax();
// 	}
//     if (TPosSource) TPos = *TPosSource;
//     if (tpos != TPos) {
//         tpos = TPos;
//         Motion->Vel = Vel; Motion->Acc = Acc; Motion->Dec = Dec; Motion->Jerk = Jerk;
// 		new(Motion) TrapezoidalMotion(tpos, 0);
//     }
//     Motion->Tick();
//     if (Mode & SM_MOTION) {
//         RPos = Motion->RPos; RVel = Motion->RVel; RAcc = Motion->RAcc; RJerk = Motion->RJerk;
//     }
//     Encoder.Tick();
// //    FPos = Encoder.FPos; FVel = Encoder.FVel; FFVel = Encoder.FFVel; FAcc = Encoder.FAcc;
// //    if (Mode & SM_ANALOGINPUT) Srvtp[0] = InputAdc.Percent;    
// 	if (Enabled() && !InTransition) {
//         if (Mode & SM_MOTION) {
//             PIn = In;
//         } else {
//             PIn = RPos;
//         }
//         if (Mode & SM_NOPOSITIONLOOP) {
//             POut = 0;
//         } else {
//             Pe = Ploop.In = PIn-FPos;
//             Ploop.Tick();
//             POut = Ploop.Out;
//         }
//         VIn = POut + RVel;
//         if (Mode & SM_NOVELOCITYLOOP) {
// //            VOut = VIn;
//         } else {
//             Vloop.In = Ve = VIn-FVel;
//             Vloop.Tick();
//             Out = Vloop.Out;
//         } 
// 		float out = (Out > DOL) ? DOL : (Out < -DOL) ? -DOL : Out;
//         out += DOffs;
//         Pwm.In = out;
// 	}
// 	Pwm.Tick();
    Io.UpdateOutputs();
}

uint32_t ServoStruct::SafetyBits() {
    uint32_t saf = 0;
    if ((Io.Io & IO_MOTORLOST) != 0) saf |= SB_MOTORLOST; 
    if ((Io.Io & IO_EMERGENCY) != 0) saf |= SB_EMERGENCY; 
    if (Motor == 0) saf |= SB_UNKNOWNMOTOR;
    if (!InitialCounter) {
        for (int i=2; i<12; i++) {
            uint32_t m = 1 << (i+14);
            if ((SafetyMask & m) == 0) 
                if ((Adc.Ain[i]<Adc.AinMin[i]) || (Adc.Ain[i]>Adc.AinMax[i])) saf |= m;
        }
    }
    return saf;
}

float* GetSignalSource(uint8_t ind) {
    return (ind == 10) ? &Signals[0].Sgn : (ind == 11) ? &Signals[1].Sgn : 0;
}
void ServoTick() {
    for (int i = 0; i < NAX; i++) Servo[i].Tick();
}
void ServoInit() {
    for (int i = 0; i < NAX; i++) Servo[i].Init(i);
}






















