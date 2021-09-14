// Servo.h
// Servo loop
// COPYRIGHT 2012 Sasha Chrichov

#pragma once

#include <math.h>
#include "Filters.h"
#include "Encoder.h"

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

// Analog inputs
//   0      User_Analog_In	PC4	ADC12_IN14
//   1      ID_OUT	PA2	ADC123_IN2
//   2      OV1	PA3	ADC123_IN3
//   3      24V_CHECK	PA5	ADC12_IN5
//   4      3V3_CHECK	PA6	ADC12_IN6
//   5      5V_CHECK	PA7	ADC12_IN7
//   6      12V_CHECK	PC5	ADC12_IN15
//   7	    24VCM	PC2	ADC123_IN12
//   8      24VDC_CM	PC3	ADC123_IN13
//   9      ADC_AIN_backup	PB0	ADC12_IN8
//   10     Temperature sensor  ADC1_IN16
//   11     ADC_0.5_VREF	PB1	ADC12_IN9

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

// Servo modes
#define SM_ENABLE           0x00000001
#define SM_PWM              0x00000002
#define SM_COMMUTATION      0x00000008
#define SM_VECTORCONTROL    0x00000010
#define SM_POSITIONLOOP     0x00000020  
#define SM_VELOCITYLOOP     0x00000040 
#define SM_CURRENTLOOP      0x00000080  
#define SM_SIMULATION       0x10000000
#define SM_HOME             0x40000000
#define SM_MOTION           0x80000000

class MotionBase;

class ServoStruct {
public:
	// uint8_t SetOffset;
    uint8_t InTransition;
    uint8_t InMotion;
    uint8_t Index;
    // uint8_t ServoVelMode;
    // uint8_t ServoPosMode;
    // uint8_t DisableDin;

    uint32_t RState, FState;
    // Bits:    
    // 0 - Enable
    // 3 - Enable Phaser 
    // 4 - Enable Current Loop 
    // 5 - Enable Velocity Loop
    // 6 - Enable Position Loop
    // 28 - Simulation
    // 31 - In Motion        

    uint16_t Error;
    uint32_t Safety;
    uint32_t SafetyRaw;
    uint32_t SafetyMask;
    
    uint8_t TPosRout;
    uint8_t TVelRout;
    uint8_t RPosRout;
    uint8_t RVelRout;
    float* TPosSource;
    float* TVelSource;
    float* RPosSource;
    float* RVelSource;

    int cntrI;
    float In, Out, Cntr[20];
    float InScale;
    float NormalOffset, LinearOffset, DcOffset;
    float Vel, Acc, Dec, KDec, Jerk;
    float TPos, TVel;
    float RPos, RVel, RAcc, RJerk, RCur;
    float FPos, FVel, FFVel, FAcc, FJerk, FCur, FCur1;
    float Pe, Ve;
    float PIn, VIn, CIn, POut, VOut;
    float Cq, Cd;
    float PeLimit;
    float Teta;
    // float DOffs, DOL;
    // float OTL;
	
	float Srvtp[10];
    float OutMin, OutMax;

    uint32_t InitialCounter;

//    EncoderStruct* Encoder;
   	MotionBase* Motion;
    PositionLoopStruct Ploop;
    VelocityLoopStruct Vloop;

    void Init(uint8_t index);
    void Tick();

    uint32_t SafetyBits();

    uint32_t UhrWrite(float v) {
        if ((v < 0.0F) || (v > 100.0F)) return MRE_WRONGVALUE;
        uhr = (uint16_t)(v*0.01*uhrPeriod + 0.5F);
        return 0;
    }
    float UhrRead() {return (float)uhr/uhrPeriod*100;}
    uint32_t UhrPeriodWrite(float v) {
        if ((v < 0.0F) || (v > 100.0F)) return MRE_WRONGVALUE;   // 100 msec maximum
        float u = UhrRead(); 
        uhrPeriod = (uint16_t)(v*TICKS_IN_MILLISECOND + 0.5F);
        UhrWrite(u);
        uhrCounter = 0;
        return 0;
    }
    float UhrPeriodRead() {return uhrPeriod*MILLISECONDS_IN_TICK;}
    uint8_t IdentifyMotor();
    uint16_t GetError(uint32_t safety);
    void SetError(uint16_t error);
    void SetMinMax();
    void Enable(uint8_t en);
    int32_t SetPos(float pos) { RPos = pos; RState = (RState & ~(SM_MOTION)) | (SM_POSITIONLOOP|SM_VELOCITYLOOP|SM_CURRENTLOOP|SM_PWM|SM_ENABLE); return 1;}
    int32_t SetVel(float vel) { VIn = vel; RState = (RState & ~(SM_MOTION|SM_POSITIONLOOP)) | (SM_VELOCITYLOOP|SM_CURRENTLOOP|SM_PWM|SM_ENABLE); return 1;}
    int32_t SetCur(float cur) { CIn = cur; RState = (RState & ~(SM_MOTION|SM_POSITIONLOOP|SM_VELOCITYLOOP)) | (SM_CURRENTLOOP|SM_PWM|SM_ENABLE); return 1;}
    int32_t SetCurQ(float cur) { Cq = cur; Cd = 0; RState = (RState & ~(SM_MOTION|SM_POSITIONLOOP|SM_VELOCITYLOOP|SM_CURRENTLOOP)) | (SM_PWM|SM_ENABLE); return 1;}
    int32_t SetTeta(float teta) { Teta = teta; RState = RState & ~SM_COMMUTATION; return 1;}
private:
  	// Inputs
    uint32_t io;
	// Current mode
	// uint8_t enabled;
	// uint8_t linearMode;
	// uint8_t dcMode;
	// // State variables
    // uint8_t uhrPulse;
    uint16_t uhr;
    uint32_t mode;
    uint16_t uhrPeriod;
    uint16_t uhrCounter;
    float tpos, tvel;
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
