// Servo.cpp
// Servo loop
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"
#include "math.h"
#include <new>

#include "global.h"
#include "adc.h"
#include "siggen.h"
#include "motion.h"
#include "servo.h"

ServoStruct Servo[NAX];

uint32_t errors[33] = {
    1000,       // FLT_MOTIONTIMEOUT
    1000, 
    1000, 
    1000, 
    1000, 
    1000, 
    1000, 
    1000,
    1000,
    1000,
    1000,
    1000,
    1000,
    1000, 
    1000, 
    1000,       // FLT_UNKNOWN 
    1020,       // FLT_GATEUVLO     
    1021,       // FLT_GATETHERMAL 
    1022,       // FLT_GATEVDS 
    1023,       // FLT_GATERESET 
    1024,       // FLT_GATE 
    1025,       // FLT_GATEI2C 
    1010,       // FLT_OVERCURRENT
    1011,       // FLT_POWERVOLTAGE
    1001,       // FLT_POSLIMIT
    1001,       // FLT_POSLIMIT
    1002,       // FLT_POSERROR
    1003,       // FLT_SOFTLIMIT
    1000,
    1000,
    1000,
    1000,
    1000,
};

void ServoStruct::Init(uint8_t index) {
    Index = index;
    Motion = NewMotion(index);
    InitialCounter = 10000;
    Vel = 10.F; Acc = 100.F; Dec = 100.F; KDec = 100.F; Jerk = 1000.F;
    RResolution = 5.6340E-5F; 
    LResolution = 0.001171875F;
    CommutPeriod = 4096;
    Ploop.Pi.Kp = 50; Ploop.Pi.Ki = 0; Ploop.Pi.Li = 0;
    Vloop.Pi.Kp = 500; Vloop.Pi.Ki = 150; Vloop.Pi.Li = 60;
    Cdloop.Pi.Kp = Cqloop.Pi.Kp = 0.2; Cdloop.Pi.Ki = Cqloop.Pi.Ki = 1000; Cdloop.Pi.Li = Cqloop.Pi.Li = 80;
    float bq[] = {100.0F, 0.7F};    // Bandwidth 700 Hz, Damping 0.7
    for (int i = 0; i < 4; i++) {
    	Vloop.Bq[i].P0 = 700.F; Vloop.Bq[i].P1 = 0.7F;
    	Vloop.Bq[i].Config(0);
    }
    CurL = 50.F;
    PwmL = 80.0F;
    EncDiL = 0.05F;
    PeL = 0;
    NsL = -50; PsL = 50;
    OtL = 2; MtL = 0;
    VelF = 0.9;
    FaultMask = 0xF808FFFF;
    FaultDisable = 0x07FF0000; 
    RelatedAxes = (1 << NAX) - 1;
}

uint32_t ServoStruct::GetError(uint32_t fault, uint8_t severity) {
    uint32_t f = fault & ((severity == 3) ? FaultDisable : (severity == 1) ? FaultKill : 0);
    for (int i=0, j=1; i<32; i++, j<<1) if (f & j) return errors[i];
    return FLT_UNKNOWN;
}
void ServoStruct::SetError(uint32_t error, uint8_t severity) { 
    Error = error; Severity = severity; 
    if (Severity > 0) {
        SetSignalRout(0, 0);
        RState &= ~1;
    }
}

void ServoStruct::Tick() {
    RPos4 = RPos3; RPos3 = RPos2; RPos2 = RPos1; RPos1 = RPos;
    Pe = RPos4 - ((RState & SM_FPOSROTARY) ? FPos1 : FPos);
    float v = (RState & SM_FVELLINEAR) ? FPos - fpos : FPos1 - fpos1; 
    FVel = v * TICKS_IN_SECOND;
    float fv = IsNan(FFVel) ? FVel : VelF * FFVel + (1.F-VelF) * FVel;
    FAcc = (fv - FFVel) * TICKS_IN_SECOND;
    FFVel = fv;
    fpos = FPos; fpos1 = FPos1; 
    Ve = VIn + RVel - FVel;
    if (TPosSource) {
        if (IsEnabled()) SetTPos(*TPosSource); else { TPosRout = 0; TPosSource = 0; }
    }
    if (--InitialCounter == 0) ResetError();
    if ((RState & 1) != enable) {
        enable = RState & 1;
        if (enable) ResetError();     
    }
    if (IsPosLoopEnabled()) {
        if (tpos != TPos) {
            tpos = TPos;
            Motion->Vel = Vel; Motion->Acc = Acc; Motion->Dec = Dec; Motion->Jerk = Jerk;
            Motion->RPos = RPos; Motion->RVel = RVel;
            new(Motion) TrapezoidalMotion(tpos, 0);
            RState |= SM_ENABLE|SM_POSITIONLOOP|SM_MOTION;
            OperationCounter = floor(MtL * TICKS_IN_SECOND);
        }
    } else {
        tpos = TPos = RPos = (RState & SM_FPOSROTARY) ? FPos1 : FPos;
        Pe = RVel = 0;
    }
    ((uint16_t*)&PreFault)[1] = ((uint16_t*)&FState)[1];
    if (Index == 0) {
        if (PreFault) {
            Fault |= PreFault & ~FaultMask;
        }
    } else {
        if (PreFault) {
            Fault |= PreFault & ~FaultMask;
        }
    }
    uint32_t severity = (Fault & FaultDisable) ? 3 : (Fault & FaultKill) ? 1 : 0;
    if (severity > Severity) {
        SetError(GetError(Fault, severity), severity);
        for (int i=0, j=1; i < NAX; i++, j<<1) if (RelatedAxes & j) Servo[i].SetError(FLT_INDUCED, severity);
    }
    Motion->Tick();
    if (RState & SM_MOTION) {
        RPos = Motion->RPos; RVel = Motion->RVel; RAcc = Motion->RAcc; RJerk = Motion->RJerk;
        if (Motion->phase == 0) RState &= ~SM_MOTION;
        if (OperationCounter && (--OperationCounter == 0)) Fault |= FLTB_MOTIONTIMEOUT;
    } else {
        if (OperationCounter && (--OperationCounter == 0)) RState &= ~SM_ENABLE;
    }
    if (VInSource) {
        if (IsEnabled()) VIn = *VInSource; else { VInRout = 0; VInSource = 0; }
    }
    if (CInSource) {
        if (IsEnabled()) SetCur(*CInSource); else { CInRout = 0; CInSource = 0; }
    }
}

uint8_t ServoStruct::SetServoMode(uint32_t mode) {
    if (mode & SM_ENABLE) {
        if ((mode & (SM_POSITIONLOOP|SM_VELOCITYLOOP)) && !(RState & SM_COMMUTATION)) return MRE_NOCOMMUT;
    }
    if (!(RState & SM_ENABLE) && (mode & SM_ENABLE)) SetError(0, 0);
    RState = (RState & ~(SM_POSITIONLOOP|SM_VELOCITYLOOP|SM_CURRENTLOOP|SM_PWM)) | mode;
    return 0;
}

uint8_t ServoStruct::SetTPos(float pos) {
    uint8_t r;
    if ((RState & (SM_POSITIONLOOP|SM_ENABLE)) != (SM_POSITIONLOOP|SM_ENABLE)) {
        if (r = SetServoMode(SM_POSITIONLOOP|SM_ENABLE)) return r;
    }
    TPos = pos;
    return 0;
}

const uint32_t romodes[] = { SM_POSITIONLOOP, SM_POSITIONLOOP, SM_VELOCITYLOOP, SM_VELOCITYLOOP, SM_CURRENTLOOP };
uint8_t ServoStruct::SetSignalRout(uint8_t var, uint8_t rout) {
    TPosRout = RPosRout = TVelRout = VInRout = CInRout = 0;
    TPosSource = RPosSource = TVelSource = VInSource = CInSource = 0;
    if ((var > 0) && (rout > 0)) {
        uint8_t r = SetServoMode(romodes[var-1] | SM_ENABLE);
        if (r) return r;
        uint8_t* const routs[] = { &TPosRout, &RPosRout, &TVelRout, &VInRout, &CInRout };
        float** const sources[] = { &TPosSource, &RPosSource, &TVelSource, &VInSource, &CInSource };
        *routs[var - 1] = rout;
        *sources[var - 1] = GetSignalSource(rout);
    }
    return 0;
}
float* ServoStruct::GetSignalSource(uint8_t ind) {
    return (ind == 10) ? &Signals[0].Sgn : (ind == 11) ? &Signals[1].Sgn : 0;
}

void ServoStruct::SetFpos(float pos) {
    TPos = tpos = RPos = FPos = FPos1 = fpos = fpos1 = pos;
    RState |= SM_SETFPOS;
}

