#pragma once

class AdcStruct {
public: 
    float Ain[12];
    static float AinMin[12];
    static float AinMax[12];
    static float Filter[12];
    uint8_t first;    
    
    void Init();
    void Tick();
    void SetMaxCurrent24V(float ampers);
private:
//    static float filter1[12];
};

extern AdcStruct Adc;