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
#define SM_PWM              0x00000002
#define SM_COMMUTATION      0x00000008
#define SM_VECTORCONTROL    0x00000010
#define SM_POSITIONLOOP     0x00000020  
#define SM_VELOCITYLOOP     0x00000040 
#define SM_CURRENTLOOP      0x00000080  
#define SM_INDEX            0x01000000
#define SM_INDEX1           0x02000000
#define SM_SIMULATION       0x10000000
#define SM_HOME             0x40000000
#define SM_MOTION           0x80000000

class MotionBase;

class ServoStruct {
public:
    uint8_t Index;
    uint8_t InTransition;
    uint32_t RState, FState;
    // Bits:    
    // 0 - Enable
    // 1 - Enable PWM        
    // 2 - Enable DC (ignored)
    // 3 - Enable Phaser
    // 4 - Enable Vector Control
    // 5 - Enable Position Loop
    // 6 - Enable Velocity Loop
    // 7 - Enable Current Loop 
    // 8 - Enable Analog Input (ignored)
    // 16 - PWM mode (ignored)
    // 17 - UHR (ignored)
    // 28 - Simulation
    // 31 - Motion Generation        

    uint16_t Error;
    uint32_t Safety;
    uint32_t SafetyRaw;
    uint32_t SafetyMask;
    uint32_t OperationCounter;

    float Vel, Acc, Dec, KDec, Jerk;    // Motion parameters
    float TPos, TVel;                   // Target values
    float RPos, RVel, RAcc, RJerk;      // Reference values
    float FPos, FPos1, FVel1, FAcc1;    // Feedback values
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
    
    uint8_t TPosRout, TVelRout, RPosRout, RVelRout;
    float *TPosSource, *TVelSource, *RPosSource, *RVelSource;
   
    uint32_t InitialCounter;

    MotionBase* Motion;
    EncoderStruct REncoder, LEncoder;
    PositionLoopStruct Ploop;
    VelocityLoopStruct Vloop;
    CurrentLoopStruct Cqloop, Cdloop;
    CommutationPhaseStruct Commut;
    VectorTransformStruct Direct, Feedback;
//    PwmStruct Pwm;

	void Init(uint8_t index);
    void Tick();
    uint32_t SafetyBits();
    uint16_t GetError(uint32_t safety);
    void SetError(uint16_t error);
    void Enable(uint8_t en);
    void SetMotionState(uint8_t stat) { if (stat) RState |= SM_MOTION; else RState &= ~SM_MOTION; }
    void SetOlCounter() { OperationCounter = (uint32_t)ceil(OtL * TICKS_IN_SECOND); }
    int32_t SetPos(float pos) { RPos = pos; RState = (RState & ~(SM_MOTION)) | (SM_POSITIONLOOP|SM_VELOCITYLOOP|SM_CURRENTLOOP|SM_PWM|SM_ENABLE); SetOlCounter(); return 1;}
    int32_t SetVel(float vel) { VIn = vel; RState = (RState & ~(SM_MOTION|SM_POSITIONLOOP)) | (SM_VELOCITYLOOP|SM_CURRENTLOOP|SM_PWM|SM_ENABLE); SetOlCounter(); return 1;}
    int32_t SetCur(float cur) { CIn = cur; RState = (RState & ~(SM_MOTION|SM_POSITIONLOOP|SM_VELOCITYLOOP)) | (SM_CURRENTLOOP|SM_PWM|SM_ENABLE); SetOlCounter(); return 1;}
    int32_t SetCurQ(float cur) { CqOut = cur; CdOut = 0; RState = (RState & ~(SM_MOTION|SM_POSITIONLOOP|SM_VELOCITYLOOP|SM_CURRENTLOOP)) | (SM_PWM|SM_ENABLE); SetOlCounter(); return 1;}
    int32_t SetTeta(float teta) { Teta = teta; RState = RState & ~SM_COMMUTATION; return 1;}

private:
    float tpos;
};

extern ServoStruct Servo[];

float* GetSignalSource(uint8_t ind);

void ServoTick();
void ServoInit();

#define ServoSetVar(func) \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < NAX); i++,j++) if (!IsNan(*buf)) Servo[j].func(*(float*)buf++); \
        return i; \
    }
#define ServoSetPos  ServoSetVar(SetPos)
#define ServoSetVel  ServoSetVar(SetVel)
#define ServoSetCur  ServoSetVar(SetCur)
#define ServoSetCurQ  ServoSetVar(SetCurQ)
#define ServoSetTeta  ServoSetVar(SetTeta)
