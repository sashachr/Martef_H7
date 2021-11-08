
#pragma once


class FilterStruct {
public:
    uint32_t Enable;
    float In, Out;
    virtual void Reset() {}
    virtual void Set(float v) {}
    virtual void Tick() {}
};

class PiStruct : public FilterStruct {
public:
	float Kp, Ki, Li;
	float KiPerTick;
	float Integral;
    PiStruct() { Integral = 0; }
    virtual void Reset() { Integral = 0; }
    virtual void Set(float v) { Integral = v; }
    virtual void Tick() {
        if (Enable) {
            float p = In*Kp;
            float i = Ki? Integral+p*KiPerTick : 0;	// .KI*SECONDS_IN_TICK : 0;
            if (fabs(i)>Li) i = (i>0)? Li : -Li;
            Integral = i;
            Out = p+i;
        } else {
            Out = In;
        }
    }
    void Config(float kp, float ki, float li) {
        Kp = kp; Ki = ki; Li = li;
        KiPerTick = ki * SECONDS_IN_TICK;
    }
};

// Biquad modes
#define BQ_LPF      0
#define BQ_NOTCH    1
#define BQ_FULL     2
#define BQ_RAW      10

class BiQuadStruct : public FilterStruct {
public:
    uint32_t Mode;
    float S1, S2;
    float A1, A2, B0, B1, B2; 
    float P0, P1, P2, P3, P4;     

    BiQuadStruct() {Enable = Mode = 0; S1 = S2 = 0;}

    void Reset(float s1, float s2) {S1 = s1; S2 = s2;}
    virtual void Reset() {Reset(In, In);}
    virtual void Tick() {
        if (Enable) {
            float s0 = In - A1*S1 - A2*S2;
            Out = B0*s0 + B1*S1 + B2*S2;
            S2 = S1;
            S1 = s0;
        } else {
            Out = S1 = S2 = In;
        }
    }
    int32_t Config(float* p) {A1 = p[0]; A2 = p[1]; B0 = p[2]; B1 = p[3]; B2 = p[4]; Reset(); return 1;}
    int32_t Config(int mode) {
        Mode = mode;
        if ((Mode < 0) || (Mode > 2)) return 0;
        switch (Mode) {
            case 0: {       // Low-pass
                float bandwidth = P0;
                float damping = P1;
                float om = 2*F_PI*bandwidth*SECONDS_IN_TICK;
                float a0 = 4 + 4*damping*om + om*om;
                A1 = (-8 + 2*om*om)/a0;   
                A2 = (4 - 4*damping*om + om*om)/a0;  
                B0 = om*om/a0;          
                B1 = 2*B0;      
                B2 = B0;          
                break;
            }
            case 1: {       // Notch
                float res = P0;
                float width = P1;
                float att = P2;
                float q = res / width;
                float om = 2*F_PI*res*SECONDS_IN_TICK;
                float a0 = 4 + 2*att/q*om + om*om;
                A1 = (-8 + 2*om*om)/a0;   
                A2 = (4 - 2*att/q*om + om*om)/a0;
                B0 = (4 + 2/q*om + om*om)/a0;    
                B1 = (-8 + 2*om*om)/a0;       
                B2 = (4 - 2/q*om + om*om)/a0; 
                break;
            }
            case 2: {       // Full
                float fp = P0;
                float dp = P1;
                float fz = P2;
                float dz = P3;
                float omp = 2*F_PI*fp*SECONDS_IN_TICK;
                float omz = 2*F_PI*fz*SECONDS_IN_TICK;
                float k = (omp*omp)/(omz*omz);
                float a0 = 4 + 4*dp*omp + omp*omp;
                A1 = (-8 + 2*omp*omp)/a0;   
                A2 = (4 - 4*dp*omp + omp*omp)/a0;
                B0 = (4 + 4*dz*omz + omz*omz)/a0*k;
                B1 = (-8 + 2*omz*omz)/a0*k;
                B2 = (4 - 4*dz*omz + omz*omz)/a0*k;  
                break;
            }
        }
        Reset();
		return 1;
    }
};

class CommutationPhaseStruct : public FilterStruct {
public:
	int32_t Incr;
	int32_t Period;
	int32_t Count;
	float CountToPhase;
    float Sin, Cos;
    void Tick() {
        Count += Incr;
        if (Count<0) Count += Period; else if (Count>=Period) Count -= Period;
        if (Enable) Out = Count*CountToPhase;
    }
    void Set(float phase) {
        Count = round(phase/CountToPhase);
        Out = phase;
        Enable = 1;
    }
    void SetPeriod(int32_t period) {
        Period = period;
        CountToPhase = 2*F_PI/period;
        Count = 0;
        Out = 0;
    }
};

struct VectorTransformStruct {
public:
	float Ia, Ib, Ic;
	float Ialpha, Ibeta;
	float Id, Iq;
	float Phase, Sin, Cos;
    void Clark() { Ialpha = Ia; Ibeta = Ia*0.577350269F+Ib*1.154700538F; } 
    void Park() { Id = Ialpha*Cos+Ibeta*Sin; Iq = -Ialpha*Sin+Ibeta*Cos; }
    void iPark() { Ialpha = Id*Cos-Iq*Sin; Ibeta = Id*Sin+Iq*Cos; }
    void iClark() { Ia = Ialpha; Ib = -Ialpha*0.5F+Ibeta*0.866025404F; Ic = -Ia-Ib; }
    void Direct() { iPark(); iClark(); }
    void Feedback() { Clark(); Park(); } 
    void SetPhase(float phase) { } 
};


