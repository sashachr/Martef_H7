
#pragma once

#define PI  3.1415926535F

// class PiStruct {
// public:
//     float In, Out;
//     float Integral;
//     float Kp, Ki, Li;      

//     PiStruct() {Kp = 1; Ki = 0.3; Li = 100; Integral = 0;}

//     uint32_t KiWrite(float v) {Ki = v; ki = v*SECONDS_IN_TICK; return 0;}
//     uint32_t LiWrite(float v) {if ((v<0)||(v>100)) return MRE_WRONGVALUE; Li = v; return 0;}

//     void Init() {Reset();}
//     void Tick() {
//         float P = In*Kp;
//         float I = Ki? Integral+P*ki : 0;
//         if (fabsf(I)>Li) I = (I>=0)? Li : -Li;
//         Integral = I;
//         Out = P + I;
//     }
//     void Reset(float Init) {Integral = Init;}
//     void Reset() {Reset(0);}
   
// private:
//     float ki;
// };

// Biquad modes
#define BQ_RAW      0
#define BQ_LPF      1
#define BQ_NOTCH    2
#define BQ_FULL     3

class FilterStruct {
public:
    uint8_t Enable;
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
class BiQuadStruct : public FilterStruct {
public:
    uint8_t Mode;
    float S1, S2;
    float A1, A2, B0, B1, B2;      

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
            Out = In;
        }
    }
    int32_t Config(float* p) {A1 = p[0]; A2 = p[1]; B0 = p[2]; B1 = p[3]; B2 = p[4]; Reset(); return 1;}
    int32_t Config(int mode, float* p) {
        if ((mode<0)||(mode>3)) return 0;
        Mode = mode;
        switch (mode) {
            case BQ_RAW: Config(p); break;
            case BQ_LPF: {
                float bandwidth = p[0];
                float damping = p[1];
                float om = 2*PI*bandwidth*SECONDS_IN_TICK;
                float a0 = 4 + 4*damping*om + om*om;
                A1 = (-8 + 2*om*om)/a0;   
                A2 = (4 - 4*damping*om + om*om)/a0;  
                B0 = om*om/a0;          
                B1 = 2*B0;      
                B2 = B0;          
                break;
            }
            case BQ_NOTCH: {
                float res = p[0];
                float width = p[1];
                float att = p[2];
                float q = res / width;
                float om = 2*PI*res*SECONDS_IN_TICK;
                float a0 = 4 + 2*att/q*om + om*om;
                A1 = (-8 + 2*om*om)/a0;   
                A2 = (4 - 2*att/q*om + om*om)/a0;
                B0 = (4 + 2/q*om + om*om)/a0;    
                B1 = (-8 + 2*om*om)/a0;       
                B2 = (4 - 2/q*om + om*om)/a0; 
                break;
            }
            case BQ_FULL: {
                float fp = p[0];
                float dp = p[1];
                float fz = p[2];
                float dz = p[3];
                float omp = 2*PI*fp*SECONDS_IN_TICK;
                float omz = 2*PI*fz*SECONDS_IN_TICK;
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

extern PiStruct PiPool[3];
extern BiQuadStruct BqPool[4];