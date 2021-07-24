#include "chip.h"

#include <string.h>
#include <math.h>

#include "Global.h"
#include "Pwm.h"

PwmStruct Pwm(TIM1);

// HR AB4 PWM transform
#define nHrAb4points		11
static const float hrAb4in[nHrAb4points] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
static const float hrAb4act[nHrAb4points] = {0, 29, 43.1, 53.7, 62.7, 70.6, 78, 84.5, 89, 94.4, 96};
static const float hrAb4inv[nHrAb4points] = {0, 29*0.25, 43.1*0.25, 53.7*0.25, 62.7*0.25, 70.6*0.25, 78*0.25, 84.5*0.25, 89*0.25, 94.4*0.25, 96*0.25};
static const float hrAb4frq[nHrAb4points] = {39600, 39600, 39600, 39600, 39600, 39600, 39600, 39600, 39600, 39600, 39600};

// HR AB5 PWM transform
#define nHrAb5points		2
static const float hrAb5in[nHrAb5points] = {0, 100};
static const float hrAb5act[nHrAb5points] = {27.0, 81.0};
static const float hrAb5inv[nHrAb5points] = {27.0, 13.0};
static const float hrAb5frq[nHrAb5points] = {43550, 39900};

// SE AB4 PWM transform
#define nSeAb4points		20
static const float seAb4in[nSeAb4points] = {0, 4.5, 9, 13.5, 18, 22.5, 27, 31.5, 36, 40.5, 45, 49.5, 54, 58.5, 63, 67.5, 72, 80, 90, 100};
static const float seAb4act[nSeAb4points] = {0, 18.52, 29.68, 38.29, 45.05, 50.5, 55.08, 59.13, 62.87, 66.48, 70.03, 73.57, 77.07, 80.49, 83.76, 86.82, 89, 93, 95, 97};
static const float seAb4inv[nSeAb4points] = {0, 2.32, 3.71, 4.79, 5.63, 6.31, 6.88, 7.39, 7.86, 8.31, 8.75, 9.2, 9.63, 10.06, 10.47, 10.85, 11.13, 11.63, 11.88, 12.13};
static const float seAb4frq[nSeAb4points] = {48800, 48800, 48716, 47569, 46836, 46351, 46011, 45759, 45562, 45404, 45275, 45171, 45089, 45028, 44985, 44956, 44941, 44926, 44923, 44923};

// SE AB5 PWM transform
#define nSeAb5points		2
static const float seAb5in[nSeAb5points] = {0, 100};
static const float seAb5act[nSeAb5points] = {37.0, 95.0};
static const float seAb5inv[nSeAb5points] = {37.0, 10.0};
static const float seAb5frq[nSeAb5points] = {48400, 44900};

void PwmTransformStruct::Init(uint32_t n, float* in, float* aout, float* iout, float* freq) {
    // Transform precalculation
    Npoints = n;
    memcpy(In, in, n<<2);
    memcpy(Aout, aout, n<<2);
    memcpy(Iout, iout, n<<2);
    memcpy(Freq, freq, n<<2);
    UpdateTable(AllTables);
}  

uint32_t PwmTransformStruct::SetTable(uint8_t table, uint16_t ind, int16_t count, uint32_t* buf) {
    if (table == NPOINTS) {
        if ((ind > 0) || (count != 1)) return 0;
        Npoints = *buf;
        updateTable |= AllTables;
    } else {
        if ((count<0)||(ind+count>MaxPwmPoints)) return 0;
        float* t = (table == InTable) ? In : (table == AoutTable) ? Aout : (table == IoutTable) ? Iout : Freq;
        memcpy(t+ind, buf, count<<2);
        updateTable |= (table == InTable) ? AllTables : table;
    }
    return count;
}

void PwmTransformStruct::CalculateFactors(float* table, float* base, float* factor) {
    for (uint32_t i=0; i<Npoints-1; i++) {
        factor[i] = (table[i+1]-table[i])/(In[i+1]-In[i]); 
        base[i] = table[i]-factor[i]*In[i];
    }
}
void PwmTransformStruct::CalculateDividerFactors() {
    float div[MaxPwmPoints];
    for (uint32_t i=0; i<Npoints; i++) {
        float f = (Freq[i] > 10000) ? Freq[i] : 10000;
        div[i] = APB2_RATE/f*0.5F;
    }
    CalculateFactors(div, pbd, pkd);
}    

void PwmTransformStruct::UpdateTable(uint8_t tables) {
    if (tables & AoutTable) CalculateActiveFactors();
    if (tables & IoutTable) CalculateInverseFactors();
    if (tables & FreqTable) CalculateDividerFactors();
    if (tables & (NPOINTS|InTable)) SetSegment(0); else SetSegment(currentSegment);
} 

void PwmTransformStruct::Transform(float input) {
    if (updateTable != 0) {UpdateTable(updateTable); updateTable = 0;}
    float in = fabsf(input);
    if (in > 100) in = 100;
    if ((in < from) || (in > to)) {
        uint8_t i = currentSegment;
        if (in < from) while (in < In[--i]) ; else while (in > In[++i + 1]) ;
        SetSegment(i);
    }
    outA = ba + ka*in, outI = bi + ki*in; 
    Period = lroundf(bd + kd*in);
    if (input >= 0) {
        Phase1 = outA; Phase2 = outI;
    } else {
        Phase1 = outI; Phase2 = outA;
    }
    Comp1 = lroundf(Period*(1-Phase1*0.01F)); 
    Comp2 = lroundf(Period*(1-Phase2*0.01F)); 
}

void PwmStruct::Init() {
    enable = -1; Enable = 0;
    mode = -1; Mode = PWM_AB4; Tpwm = &Ab4; 
    // Hadware initialization
    RCC->APB2ENR |= 0x00000001;        // Enable TIM1 clock
    CR1 = hard->CR1 = 0x00A0;     // Center-aligned mode 1, buffered ARR
    hard->PSC = 0;          // No prescale
    hard->RCR = 0;          // No repetition
    hard->CCMR1 = 0x7878;   // Channel1/2: PWM mode 2, buffered
    hard->CCMR2 = 0x0038;   // Channel3 (pushpull): PWM mode Toggle, buffered
    hard->CCR3 = 0;         // Compare value for channel 3
    hard->CCER = 0x0F55;    // Channel1/2/3: enable outputs, channel 3: inverts outputs to enable make-before-break
    hard->BDTR = 0x0400;    // OSSI
    SetBreakBeforeMake(40); // Default break-before-make 40x6 = 240 nsec
    CR1 = hard->CR1 = 0x00A1;    // Center-aligned mode 1, buffered ARR, Enable
}  

void PwmStruct::Tick() {
    if (Motor == 0) return;
    if (mode != Mode) {mode = Mode; Tpwm = (mode == PWM_AB4) ? &Ab4 : &Ab5;};
    if (enable != Enable) {
        enable = Enable;
        SetPwm(0);
        if (enable) {
            switchCounter = -1;
        } else {
            switchCounter = TICKS_IN_MILLISECOND;
            hard->CCMR1 &= ~0x3030;    // Force inactive level on buck outputs
            hard->CCER |= 0x0088;      // Inverse for inverse outputs
        }
    } else if (InTransition()) {
        if (enable) {
            switchCounter = 0;
            hard->CCER &= ~0x0088;    // Normal polarity for inverse outputs
            hard->CCMR1 |= 0x7070;    // Restore buck PWM mode
            hard->BDTR |= 0xC000;     // Enable main output
        } else {
            if (--switchCounter == 0) {
                hard->BDTR &= ~0xC000;    // Disable main output
            }
        }
    } 
    if (enable && !InTransition()) SetPwm(In);
}

void PwmStruct::SetPwm(float control) {
    Tpwm->Transform(control);
    ARR = Tpwm->Period;
    CCR1 = Tpwm->Comp1;
    CCR2 = Tpwm->Comp2;
    hard->CR1 = CR1 | 0x0002;   // Disable update events
    hard->ARR = ARR; hard->CCR1 = CCR1; hard->CCR2 = CCR2;
    hard->CR1 = CR1;            // Enable update events
}

void PwmStruct::SetBreakBeforeMake(uint8_t bbm) {
    BreakBeforeMake = bbm;
    hard->BDTR = (hard->BDTR & 0xFF00) | bbm;
}

void PwmStruct::SetMotor(uint8_t motor) {
    // PWM transforms
    if (motor < 100) {  // HR
        Ab4.Init(nHrAb4points, (float*)hrAb4in, (float*)hrAb4act, (float*)hrAb4inv, (float*)hrAb4frq);
        Ab5.Init(nHrAb5points, (float*)hrAb5in, (float*)hrAb5act, (float*)hrAb5inv, (float*)hrAb5frq);
    } else {    // SE
        Ab4.Init(nSeAb4points, (float*)seAb4in, (float*)seAb4act, (float*)seAb4inv, (float*)seAb4frq);
        Ab5.Init(nSeAb5points, (float*)seAb5in, (float*)seAb5act, (float*)seAb5inv, (float*)seAb5frq);
    }
    SetPwm(0);
}
