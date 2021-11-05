// Servo.h
// Servo loop
// COPYRIGHT 2012 Sasha Chrichov

#pragma once

#include "filters.h"

#define N_BIQUADS	4

// Bits in the Safety variable
#define SB_MOTORLOST    0x00000001
#define SB_EMERGENCY    0x00000002
#define SB_UNKNOWNMOTOR 0x00000100
#define SB_OVERVOLTAGE  0x00010000
#define SB_24V          0x00020000
#define SB_3V3          0x00040000
#define SB_5V           0x00080000
#define SB_12V          0x00100000
#define SB_24VCM        0x00200000
#define SB_24VDCCM      0x00400000
#define SB_AIN          0x00800000
#define SB_TEMP         0x01000000
#define SB_05REF        0x02000000

class PositionLoopStruct : public FilterStruct {
public:
    PiStruct Pi;
    PositionLoopStruct() {}
    void Tick() {
        if (Enable) {
            Pi.In = In;
            Pi.Tick();
            Out = Pi.Out;
        }
    }
    void Reset() {
        Pi.Reset();
    }
};
class VelocityLoopStruct : public FilterStruct {
public:
    float Vltp[4];
    BiQuadStruct Bq[N_BIQUADS];
    PiStruct Pi;
    VelocityLoopStruct() {}
    void Tick() {
        if (Enable) {
            Vltp[0] = Bq[0].In = In;
            Bq[0].Tick();
            Vltp[1] = Bq[1].In = Bq[0].Out;
            Bq[1].Tick();
            Vltp[2] = Bq[2].In = Bq[1].Out;
            Bq[2].Tick();
            Vltp[3] = Pi.In = Bq[2].Out;
            Pi.Tick();
            Out = Pi.Out;
        }
    }
    void Reset() {
        Bq[0].In = Bq[1].In = Bq[2].In = Pi.In = Vltp[0] = Vltp[1] = Vltp[2] = Vltp[3] = In;
        Bq[0].Reset(); Bq[1].Reset(); Bq[2].Reset(); Pi.Reset();
    }
    void Set(float v) { Pi.Set(v); }
};
class CurrentLoopStruct : public FilterStruct {
public:
    float Cltp[2];
    BiQuadStruct Bq[N_BIQUADS];
    PiStruct Pi;
    void Tick() {
        if (Enable) {
            Cltp[0] = Pi.In = In;
            Pi.Tick();
            Cltp[1] = Out = Pi.Out;
        }
    }
    void Reset() {
        Cltp[0] = Cltp[1] = Pi.In = In;
        Pi.Reset();
    }
    void Set(float v) { Pi.Set(v); }
};

// Servo state bits
#define SM_ENABLE           0x00000001
#define SM_MOTION           0x00000002
#define SM_PWM              0x00000004
#define SM_COMMUTATION      0x00000008
#define SM_VECTORCONTROL    0x00000010
#define SM_POSITIONLOOP     0x00000020
#define SM_VELOCITYLOOP     0x00000040
#define SM_CURRENTLOOP      0x00000080
#define SM_SETFPOS          0x00000100
#define SM_FPOSROTARY       0x00000400
#define SM_FVELLINEAR       0x00000800
#define SM_INDEX            0x00001000
#define SM_INDEX1           0x00002000
#define SM_SIMULATION       0x00004000
#define SM_HOME             0x00008000

// Fault bits
#define FLTB_MOTIONTIMEOUT  0x00000001
#define FLTB_GATEVCCLO      0x00010000
#define FLTB_GATETHERM      0x00020000
#define FLTB_GATEVDS        0x00040000
#define FLTB_GATERESET      0x00080000
#define FLTB_GATE           0x00100000
#define FLTB_GATEI2C        0x00200000
#define FLTB_OVERCURRENT    0x00400000
#define FLTB_POWERVOLTAGE   0x00800000
#define FLTB_NLIMIT         0x01000000
#define FLTB_PLIMIT         0x02000000
#define FLTB_PEL            0x04000000

// Routable variables
#define RO_TPOS             1
#define RO_RPOS             2
#define RO_RVEL             3
#define RO_VIN              4
#define RO_CIN              5


class MotionBase;

class ServoStruct {
public:
    uint8_t Index;
    uint8_t InTransition;
    uint32_t RState;
    uint32_t FState;
    // Control state bits:    
    // 0 - Enable
    // 1 - Motion Generation        
    // 2 - Enable PWM        
    // 3 - Enable Phaser
    // 4 - Enable Vector Control
    // 5 - Enable Position Loop
    // 6 - Enable Velocity Loop
    // 7 - Enable Current Loop 
    // 12 - Index
    // 13 - Index1
    // 14 - Simulation
    // 15 - Motion Generation     
    
    uint32_t RelatedAxes;
    uint32_t PreFault, Fault, FaultMask, FaultKill, FaultDisable;
    // Fault bits
    //  0 - Motion timeout
    // 16 - Gate VCC UVLO fault
    // 17 - Gate thermal fault
    // 18 - Gate VDS fault
    // 19 - Gate reset
    // 20 - Gate Fault
    // 21 - Gate I2C fault
    // 22 - Overcurrent fault
    // 23 - Power voltage fault
    // 24 - Negative limit fault
    // 25 - Positive limit fault
    // 26 - Position error fault

    uint32_t Error, Severity;
    uint32_t Safety;
    uint32_t SafetyRaw;
    uint32_t SafetyMask;
    uint32_t OperationCounter;

    float Vel, Acc, Dec, KDec, Jerk;    // Motion parameters
    float TPos, TVel;                   // Target values
    float RPos, RVel, RAcc, RJerk;      // Reference values
    float RPos1, RPos2, RPos3, RPos4;   // Delayed RPos
    float FPos, FPos1, FVel, FAcc;      // Feedback values
    float VelF, FFVel;                  // Filtered feedback velocity
    float IndPos, IndPos1;              // Index position of rotary and linear encoders
    float Pe, Ve;                       // Position error, velocity error
    float PIn, VIn, POut, VOut;         // Inputs/outputs of position/velocity loop
    float CIn, CqOut, CdOut;            // Current Q loop input, current D/Q loop outputs 
    float FCa, FCb, FCq, FCd;           // Current A/B feedback and transformed D/Q feedback
    float OutA, OutB, OutC;             // PWM A/B/C phase values
    float CurL, PwmL;                   // Current limit (applies to CIn), PWM limit (applies to PWM A/B/C)
    float EncDiL;                       // Permitted discrepancy of encoder feedbacks
    float PeL;                          // Permitted position error
    float NsL, PsL;                     // Software limits
    float OtL, MtL;                     // Maximal time of open-loop operation and single motion
    float Teta;                         // Commutation angle
    float RResolution, LResolution;     // Resolution of rotary and linear encoders
    uint32_t CommutPeriod;                // Commutation period in rotary encoder counts

    uint8_t TPosRout, RPosRout, TVelRout, VInRout, CInRout;
    float *TPosSource, *RPosSource, *TVelSource, *VInSource, *CInSource;
   
    uint32_t InitialCounter;

    MotionBase* Motion;
    PositionLoopStruct Ploop;
    VelocityLoopStruct Vloop;
    CurrentLoopStruct Cqloop, Cdloop;
//    PwmStruct Pwm;

	void Init(uint8_t index);
    void Tick();
//    uint32_t SafetyBits();
    uint32_t GetError(uint32_t safety, uint8_t severity);
    void SetError(uint32_t error, uint8_t severity);
    void ResetError() { Fault = Error = Severity = 0; }
    uint8_t IsEnabled() { return RState & SM_ENABLE; }
    uint8_t IsPosLoopEnabled() { return (RState & SM_ENABLE) && (RState & SM_POSITIONLOOP); }
    void SetMotionState(uint8_t stat) { if (stat) RState |= SM_MOTION; else RState &= ~SM_MOTION; }
    uint8_t SetServoMode(uint32_t mode);
    void SetOlCounter() { OperationCounter = (uint32_t)ceil(OtL * TICKS_IN_SECOND); }
    uint8_t SetTPos(float pos);
    int32_t SetPos(float pos) { RPos = pos; RState = (RState & ~(SM_MOTION)) | (SM_POSITIONLOOP|SM_VELOCITYLOOP|SM_CURRENTLOOP|SM_PWM|SM_ENABLE); SetOlCounter(); return 1;}
    int32_t SetVel(float vel) { VIn = vel; RState = (RState & ~(SM_MOTION|SM_POSITIONLOOP)) | (SM_VELOCITYLOOP|SM_CURRENTLOOP|SM_PWM|SM_ENABLE); SetOlCounter(); return 1;}
    int32_t SetCur(float cur) { CIn = cur; RState = (RState & ~(SM_MOTION|SM_POSITIONLOOP|SM_VELOCITYLOOP)) | (SM_CURRENTLOOP|SM_PWM|SM_ENABLE); SetOlCounter(); return 1;}
    int32_t SetCurQ(float cur) { CqOut = cur; CdOut = 0; RState = (RState & ~(SM_MOTION|SM_POSITIONLOOP|SM_VELOCITYLOOP|SM_CURRENTLOOP)) | (SM_PWM|SM_ENABLE); SetOlCounter(); return 1;}
    int32_t SetTeta(float teta) { Teta = teta; RState = RState & ~SM_COMMUTATION; return 1;}
    uint8_t SetSignalRout(uint8_t var, uint8_t rout);
    static float* GetSignalSource(uint8_t rout);
    void SetFpos(float pos);

private:
    float tpos, fpos, fpos1, fvel1;
    uint8_t enable;
};

extern ServoStruct Servo[];


inline void ServoInit() { for (int i = 0; i < NAX; i++) Servo[i].Init(i); }
inline void ServoTick() { for (int i = 0; i < NAX; i++) Servo[i].Tick(); }

#define ServoSetVar(func) \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < NAX); i++,j++) if (!IsNan(*buf)) Servo[j].func(*(float*)buf++); \
        return i; \
    }
#define ServoSetTPos  ServoSetVar(SetTPos)
#define ServoSetPos  ServoSetVar(SetPos)
#define ServoSetVel  ServoSetVar(SetVel)
#define ServoSetCur  ServoSetVar(SetCur)
#define ServoSetCurQ  ServoSetVar(SetCurQ)
#define ServoSetTeta  ServoSetVar(SetTeta)
