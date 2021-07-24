#pragma once

#define PWM_AB4 0
#define PWM_AB5 1

#define NPOINTS     1   
#define InTable     2
#define AoutTable   0x0010
#define IoutTable   0x0020
#define FreqTable   0x0040
#define AllTables   0x0073

#define MaxPwmPoints    20

class PwmTransformStruct {
public:
    // PWM transform
    uint32_t Npoints;               // Number of points in the tables
    float In[MaxPwmPoints];         // Input table
    float Aout[MaxPwmPoints];       // Active phase output table
    float Iout[MaxPwmPoints];       // Inverse phase output table
    float Freq[MaxPwmPoints];       // Frequency table

    // Output values
    float Phase1, Phase2;           // Phase output values (percents)
    uint32_t Comp1, Comp2;          // Compare values
    uint32_t Period;                // Output period (counts)

    void Init(uint32_t n, float* in, float* aout, float* iaout, float* freq);
    uint32_t SetTable(uint8_t table, uint16_t ind, int16_t count, uint32_t* buf);
    void UpdateTable(uint8_t tables);
    void Transform(float in);

private:
    // Current PWM transform
    float from, to;     // Limits of input 
    float ba, ka;       // Linear transform for active phase
    float bi, ki;       // Linear transform for inactive phase
    float bd, kd;       // Linear transform for divider
    uint8_t currentSegment;

    void SetSegment(uint8_t s) {
        currentSegment = s; from = In[s]; to = In[s+1]; 
        ba = pba[s]; ka = pka[s]; bi = pbi[s]; ki = pki[s]; bd = pbd[s]; kd = pkd[s];
    }

    // Precalculated factors for linear interpolation 
    float pba[MaxPwmPoints-1], pka[MaxPwmPoints-1], pbi[MaxPwmPoints-1], pki[MaxPwmPoints-1], pbd[MaxPwmPoints-1], pkd[MaxPwmPoints-1];      

    float outA, outI;   // Output percents for active/inactive phases and divider

    uint8_t updateTable;
    void CalculateFactors(float* table, float* base, float* factor);
    void CalculateActiveFactors() {CalculateFactors(Aout, pba, pka);}
    void CalculateInverseFactors() {CalculateFactors(Iout, pbi, pki);}
    void CalculateDividerFactors();
};

class PwmStruct {
public:
    uint8_t Enable;
    uint8_t Mode;
    float In;
    uint8_t BreakBeforeMake;        // Break-before-make time in counts, minimal count is 1/84 MHz = 12 nsec

    // PWM transforms for AB4 and AB5 modes
    PwmTransformStruct Ab4, Ab5;
    PwmTransformStruct* Tpwm;       // Current mode transform

	PwmStruct(TIM_TypeDef* timer) : hard(timer) { Init(); }
	
    void Init();
    int32_t SetTable(uint8_t mode, uint8_t table, uint16_t ind, int16_t count, uint32_t* buf) {
        return ((mode == PWM_AB4) ? &Ab4 : &Ab5)->SetTable(table, ind, count, buf);
    }
    void SetPwm(float control);
    void SetBreakBeforeMake(uint8_t bbm);
    void SetMotor(uint8_t motor);
    void Tick();
    uint8_t InTransition() {return switchCounter != 0;}

private:
    TIM_TypeDef* hard;
    uint8_t enable, mode;
    int16_t switchCounter;

    uint32_t CR1, ARR, CCR1, CCR2;      // Hardware register mirrors
};

extern PwmStruct Pwm;

