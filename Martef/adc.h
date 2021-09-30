#pragma once

class AdcStruct {
public: 
    float Ain[24];
    static float AinMin[24];
    static float AinMax[24];
    static float Filter[24];
    uint8_t first;    
    
    void Init();
    void Tick();
    void SetMaxCurrent24V(float ampers);
private:
//    static float filter1[12];
};

extern AdcStruct Adc;