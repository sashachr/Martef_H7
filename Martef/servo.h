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
#define SM_DC               0x00000004
#define SM_NOPHASER         0x00000008
#define SM_NOCURRENTLOOP    0x00000010  
#define SM_NOVELOCITYLOOP   0x00000020  
#define SM_NOPOSITIONLOOP   0x00000040 
#define SM_LINEAR           0x00010000
#define SM_UHR              0x00020000
#define SM_ANALOGINPUT      0x01000000  
#define SM_NOMOTION         0x10000000

class ServoStruct {
public:
	uint8_t Enabled() {return (uint8_t)(Mode & SM_ENABLE);}
	uint8_t LinearMode() {return (Mode & SM_LINEAR) != 0;}
	uint8_t UhrMode() {return (Mode & SM_UHR) != 0;}
	uint8_t DcMode() {return (Mode & SM_DC) != 0;}
	// uint8_t SetOffset;
    uint8_t InTransition;
    // uint8_t Fault;

    // uint8_t ServoVelMode;
    // uint8_t ServoPosMode;
    // uint8_t DisableDin;

    uint32_t Mode;
    // Bits:    
    // 0 - Enable
    // 1 - Enable PWM        
    // 2 - Enable DC
    // 3 - Enable Phaser (ignored)
    // 4 - Enable Current Loop (ignored)
    // 5 - No Velocity Loop
    // 6 - No Position Loop
    // 16 - 0 - Normal mode, 1 - Linear mode
    // 17 - Enable UHR
    // 24 - Enable Analog Input
    // 28 - No Motion Generation        

    uint16_t Error;
    uint32_t Safety;
    uint32_t SafetyRaw;
    uint32_t SafetyMask;


    int cntrI;
    float In, Out, Cntr[20];
    float InScale;
    float NormalOffset, LinearOffset, DcOffset;
    float RPos, RVel, RAcc, RJerk;
    float FPos, FVel, FFVel, FAcc;
    float Pe, Ve;
    float PIn, VIn, POut, VOut;
    float PeLimit;
    float DOffs, DOL;
    float OTL;
	
	float Srvtp[10];
    float OutMin, OutMax;

    uint32_t InitialCounter;

//    EncoderStruct* Encoder;
    PositionLoopStruct Ploop;
    VelocityLoopStruct Vloop;

	ServoStruct();

    void Init();
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
    int32_t WriteDout(float dout);
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
};

extern ServoStruct Servo[];