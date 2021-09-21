// Servo.cpp
// Servo loop
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"
#include "math.h"
#include <new>

#include "global.h"
#include "adc.h"
#include "encoder.h"
#include "siggen.h"
#include "motion.h"
#include "servo.h"

ServoStruct Servo[NAX];

void ServoStruct::Init(uint8_t index) {
    Index = index;
    Motion = NewMotion(index);
    InitialCounter = 1000;
    Vel = 500; Acc = 5000; Dec = 5000; KDec = 5000; Jerk = 500000;
    CurL = PwmL = 100.0F;
    EncDiL = 0.05F;
    PeL = 0.05F;
    NsL = -50; PsL = 50;
    OtL = 2; MtL = 5;
    REncoder.Resolution = 0.001; LEncoder.Resolution = 0.0005;
}

void ServoStruct::Enable(uint8_t en) {}

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
    // if (error != 0) {
    //     Io.Io |= IO_FAULT;
    // } else {
    //     Io.Io &= ~IO_FAULT;
    // }
}

void ServoStruct::Tick() {
    if (TPosSource) TPos = *TPosSource;
    if (tpos != TPos) {
        tpos = TPos;
        Motion->Vel = Vel; Motion->Acc = Acc; Motion->Dec = Dec; Motion->Jerk = Jerk;
		new(Motion) TrapezoidalMotion(tpos, 0);
        RState |= SM_ENABLE|SM_PWM|SM_CURRENTLOOP|SM_VELOCITYLOOP|SM_POSITIONLOOP|SM_MOTION;
        OperationCounter = floor(MtL * TICKS_IN_SECOND);
    }
    Motion->Tick();
    if (RState & SM_MOTION) {
        RPos = Motion->RPos; RVel = Motion->RVel; RAcc = Motion->RAcc; RJerk = Motion->RJerk;
        if (Motion->phase == 0) RState &= ~SM_MOTION;
        if (--OperationCounter == 0) SetError(FLT_TIMEOUT);
    } else {
        if (OperationCounter) {
            if (--OperationCounter == 0) RState &= ~SM_ENABLE;
        }
    }

    // if (InitialCounter) InitialCounter--;
    // SafetyRaw = SafetyBits();
    // Safety = SafetyRaw & ~SafetyMask;
    // if (Safety) {
    //     if (Error == 0) SetError(GetError(Safety));
    // }
    // if (Error != 0) {
    //     FState &= ~SM_ENABLE;
    // }
    // uint32_t modechange =  RState ^ FState;
    // if (modechange) {
    //     FState =  RState;
    //     if (modechange & SM_ENABLE) Enable(FState & SM_ENABLE);
    // }
	// if ((modechange & SM_ENABLE) && !InTransition) {
	// 	if (FState & SM_ENABLE) {
    //         Pwm.Enable = 1;
    //         Ploop.Reset(); Vloop.Reset(); Cqloop.Reset(); Cdloop.Reset();
    //     } else {
    //         POut = VOut = Pwm.A = Pwm.B = Pwm.C = 0;
    //     }
	// }
    // REncoder.Tick(); LEncoder.Tick(); 
    // if (!(FState & SM_INDEX) && (REncoder.IndFlag)) {
    //     REncoder.IndFlag = 0;
    //     IndPos = REncoder.IndPos;
    //     FState |= SM_INDEX;
    // }
    // if (!(FState & SM_INDEX1) && (LEncoder.IndFlag)) {
    //     LEncoder.IndFlag = 0;
    //     IndPos1 = LEncoder.IndPos;
    //     FState |= SM_INDEX1;
    // }
    // if (FState & SM_SIMULATION) {
    //     FPos = RPos; Pe = 0; ;
    // } else {
    //     FPos = REncoder.FPos;		// change for two encoders
    //     FPos1 = REncoder.FPos; FVel1 = REncoder.FVel;
    // }
	// if ((FState & SM_ENABLE) && !InTransition) {
    //     if (FState & SM_POSITIONLOOP) {
    //         Pe = Ploop.In = RPos-FPos;
    //         Ploop.Tick();
    //         POut = VIn = Ploop.Out;
    //     }
    //     if (FState & SM_VELOCITYLOOP) {
    //         Vloop.In = Ve = VIn+RVel-FVel1;
    //         Vloop.Tick();
    //         VOut = CIn = Vloop.Out;
    //     } 
    //     if (FState & SM_COMMUTATION) {
    //         Commut.Incr = REncoder.Incr;
    //         Commut.Tick();
    //         Direct.Sin = Feedback.Sin = Commut.Sin; Direct.Cos = Feedback.Cos = Commut.Cos; 
    //         Teta = Commut.Out;
    //     } 
    //     Direct.Sin = Feedback.Sin = sin(Teta); Direct.Cos = Feedback.Cos = cos(Teta); 
    //     if (FState & SM_CURRENTLOOP) {
    //         Feedback.Ia = Adc.Ain[0]; Feedback.Ib = Adc.Ain[1];
    //         Feedback.Feedback();
    //         FCq = Feedback.Iq; FCd = Feedback.Id;
    //         float in = (CIn > CurL) ? CurL : (CIn < -CurL) ? -CurL: CIn;
    //         Cqloop.In = in - FCq;
    //         Cqloop.Tick();
    //         Cq = Cqloop.Out;
    //         Cdloop.In = 0 - FCd;
    //         Cdloop.Tick();
    //         Cd = Cdloop.Out;
    //     }
    //     if (FState & SM_PWM) {
	// 		Direct.Iq = Cq; Direct.Id = Cd;
	// 		Direct.Direct();
	// 		OutA = Direct.Ia; OutB = Direct.Ib; OutC = Direct.Ic;
	// 		float lim = (PwmL < 100.F) ? PwmL : 100.F;
	// 		if (OutA > lim) OutA = lim; else if (OutA < -lim) OutA = -lim;
	// 		if (OutB > lim) OutB = lim; else if (OutB < -lim) OutB = -lim;
	// 		if (OutC > lim) OutC = lim; else if (OutC < -lim) OutC = -lim;
	// 		Pwm.A = OutA; Pwm.B = OutB; Pwm.C = OutC;
    //     }
	// } else {
    //     Pwm.A = Pwm.B = Pwm.C = 0;
    // }
	// Pwm.Tick();
}

uint32_t ServoStruct::SafetyBits() {
    uint32_t saf = 0;
    // if ((Io.Io & IO_MOTORLOST) != 0) saf |= SB_MOTORLOST; 
    // if ((Io.Io & IO_EMERGENCY) != 0) saf |= SB_EMERGENCY; 
    // if (Motor == 0) saf |= SB_UNKNOWNMOTOR;
    // if (!InitialCounter) {
    //     for (int i=2; i<12; i++) {
    //         uint32_t m = 1 << (i+14);
    //         // if ((SafetyMask & m) == 0) 
    //         //     if ((Adc.Ain[i]<Adc.AinMin[i]) || (Adc.Ain[i]>Adc.AinMax[i])) saf |= m;
    //     }
    // }
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
