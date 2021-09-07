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

ServoStruct::ServoStruct() {
	uhrPeriod = TICKS_IN_MILLISECOND;		// 1 msec period
	uhr = (uint16_t)(0.1*TICKS_IN_MILLISECOND + 0.5F);	// 10% UHR
	uhrCounter = 0;
    mode = SM_LINEAR | SM_DC | SM_ENABLE;       // to cause initialization
    Mode = 0;
}  	

void ServoStruct::Init(uint8_t index) {
    Index = index;
    Motion = NewMotion(index);
	Pwm.Init();
//	Pwm.SetFrequency(80000);
    float bq[] = {100.0F, 0.7F};    // Bandwidth 700 Hz, Damping 0.7
    DOL = 100.0F;
    // BqDc.Config(BQ_LPF, bq);
    // BqAc.Config(BQ_LPF, bq);
    InitialCounter = 10000;         // 0.5 sec per Vlad's request
}

void ServoStruct::Enable(uint8_t en) {
    if (en) {
        if (Error) SetError(0);
        mode &= SM_ENABLE;
    } 
    Mode |= en;
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
    SafetyRaw = SafetyBits();
    Safety = SafetyRaw & ~SafetyMask;
    if (Safety) {
        if (Error == 0) SetError(GetError(Safety));
    }
    if (Error != 0) {
        Mode &= ~SM_ENABLE;
    }
    uint32_t modechange =  mode ^ Mode;
    if (modechange) {
        mode = Mode;
        if (modechange & SM_ENABLE) Enable(Mode & SM_ENABLE);
    }
    if ((modechange & SM_DC) && !InTransition) {
		if (Enabled()) {
            Pwm.Enable = (Mode & SM_DC) == 0;
            SetMinMax();
        }
    }
	if ((modechange & SM_ENABLE) && !InTransition) {
        Pwm.Enable = 0;
		if (Enabled()) {
            SetMinMax();
            Pwm.Enable = (Mode & SM_DC) == 0;
            Ploop.Reset(); Vloop.Reset();
        } else {
            POut = VOut = Pwm.In = 0;
        }
	}
	if ((modechange & SM_LINEAR) && !InTransition) {
		Pwm.Mode = LinearMode();
        if (Enabled()) SetMinMax();
	}
    if (TPosSource) TPos = *TPosSource;
    if (tpos != TPos) {
        tpos = TPos;
        Motion->Vel = Vel; Motion->Acc = Acc; Motion->Dec = Dec; Motion->Jerk = Jerk;
		new(Motion) TrapezoidalMotion(tpos, 0);
    }
    Motion->Tick();
    RPos = Motion->RPos; RVel = Motion->RVel; RAcc = Motion->RAcc; RJerk = Motion->RJerk;
    Encoder.Tick();
//    FPos = Encoder.FPos; FVel = Encoder.FVel; FFVel = Encoder.FFVel; FAcc = Encoder.FAcc;
//    if (Mode & SM_ANALOGINPUT) Srvtp[0] = InputAdc.Percent;    
	if (Enabled() && !InTransition) {
        if (Mode & SM_NOMOTION) {
            PIn = In;
        } else {
            PIn = RPos;
        }
        if (Mode & SM_NOPOSITIONLOOP) {
            POut = 0;
        } else {
            Pe = Ploop.In = PIn-FPos;
            Ploop.Tick();
            POut = Ploop.Out;
        }
        VIn = POut + RVel;
        if (Mode & SM_NOVELOCITYLOOP) {
//            VOut = VIn;
        } else {
            Vloop.In = Ve = VIn-FVel;
            Vloop.Tick();
            Out = Vloop.Out;
        } 
		float out = (Out > DOL) ? DOL : (Out < -DOL) ? -DOL : Out;
        out += DOffs;
        Pwm.In = out;
	}
	Pwm.Tick();
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

void ServoStruct::SetMinMax() {
    if (Mode & SM_DC) {
        OutMin = -100; OutMax = 100;
    } else {
        if (Mode & SM_UHR) {
            OutMin = -50; OutMax = 50;
        } else if (Motor == 132) {      // SE32
            OutMin = -85; OutMax = 85;
        } else {
            OutMin = -100; OutMax = 100;
        }
    }
}

struct MotorIdStruct {
    uint8_t nElements;
    float VIdMin, VIdMax;
};
MotorIdStruct Motors[] = {     //        max[v]	min[v]                  
    { 1,  4.24, 13.94},        // HR/SE2  0.46	0.14
    { 2, 16.36, 26.07},        // HR/SE2  0.86	0.54
    { 4, 28.48, 38.19},        // HR/SE4  1.26	0.94
    { 6, 40.60, 50.31},        // HR/SE6  1.66	1.34
    { 8, 52.72, 62.43},        // HR/SE8  2.06	1.74
    {12, 64.84, 74.55},        // HR/SE12 2.46	2.14
    {16, 76.96, 86.67},        // HR/SE16 2.86	2.54
    {32, 89.09, 98.79},        // HR/SE32 3.26	2.94
};

int32_t ServoStruct::WriteDout(float out) {
    Out = out;
    Mode |= 0x10000061;     // Enable, disable motion/position/velocity
    return 1;
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






















