// IO.H 
// Configuration of IO pins	
// COPYRIGHT 2019 Sasha Chrichov

//Input bits
#define IO_ENABLE    0x00010000
#define IO_DC        0x00020000
#define IO_LINEAR    0x00040000
#define IO_UHR       0x00080000
#define IO_SETOFFSET 0x00100000
#define IO_EMERGENCY 0x10000000
#define IO_MOTORLOST 0x20000000
#define IO_FAULT     0x80000000

class IoStruct {
public:
    uint32_t Io;            // Final input bits
    uint32_t IoRaw;         // Raw input bits (before masking and inversion)
    uint32_t IoDirection;   // 0 - input; 1 - output
    uint32_t IoInverse;   // For inputs: 0 - low level is 0, high level is 1; 1 - low level is 1, high level is 0

    void Init() {IoInverse = 0x101F0000;}
    void UpdateInputs();
    void UpdateOutputs();
    void SetInversion(uint32_t inverse);
};

extern IoStruct Io;
