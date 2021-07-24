// Encoder.h
// AqB encoder handling
// COPYRIGHT 2014 Sasha Chrichov

#pragma once

class PeriodicCounter {
public:
    int32_t Min, Max;
    int32_t Count;
    void Increment(int32_t incr) {
        Count += incr;
        if (Count < Min) Count += Max-Min; else if (Count > Max) Count -= Max-Min;
    }
    int Active() {return Max>Min;}
};
class EncoderStruct {
public:
	TIM_TypeDef* Timer;
    PeriodicCounter Periodic;
	float Resolution;
	float Offset;
	float VelFiltFact;
	int32_t LastCount;
	int32_t Incr;
	int32_t Count;
	uint16_t Error;
	uint8_t IndFlag;
	int32_t IndCount;
	float FPos, FVel, FFVel, FAcc;
	float IndPos;
    void Init() {
        RCC->APB1LENR |= 0x00000001; // TIM2 clock enable
		Resolution = 0.0001;
        Timer = TIM2;
        Timer->CCMR1 = 0x0101;      // CC1/CC2 are maped on TI1/TI2
        Timer->CCMR2 = 0x0001;      // CC3 is maped on TI3 (index)
        Timer->CCER = 0x0100;       // CC3 capture enabled
        Timer->SMCR = 0x0003;       // Encoder mode 3 (AqB inputs)
        Timer->CR1 = 0x0001;        // Enable
    }
    void Tick() {
        int32_t newCount = Timer->CNT;
        Incr = (newCount-LastCount);
        LastCount = newCount;
        Count += Incr;
        int32_t cnt = Count;
        // if (Periodic.Active()) {
        //     Periodic.Increment(Incr);
        //     cnt = Periodic.Count;
        // }
        float pos = cnt*Resolution+Offset;
        float vel = Incr*Resolution*TICKS_IN_SECOND;
        FPos = pos;
        FFVel = VelFiltFact*FFVel+(1.F-VelFiltFact)*vel;
        FVel = vel;
        if (!IndFlag && (Timer->SR & 0x0800)) {     // CC3 Capture flag
            IndCount = Timer->CCR3-LastCount+Count;
            IndPos = IndCount*Resolution+Offset;
        }
    }
    float NormalizePE(float pe) {
        if (2*pe < Resolution) return 0;
        return pe;
    }
    void ResetCapture() {
        IndFlag = 0;
        Timer->SR &= ~0x0800;
    }
};

extern EncoderStruct Encoder;

