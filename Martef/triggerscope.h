// Scope.h
// Scope support
// COPYRIGHT 2012 Sasha Chrichov

#pragma once

#define SC_MAXDEPTH			1000
#define SC_MAXSIGNALS		8

// Commands 
#define SCC_CONFIG		1
#define SCC_START		2
#define SCC_CONFIGSTART	3
#define SCC_FORCE		4
#define SCC_STOP		5
#define SCC_READ		6
#define SCC_READSTART   7

// Scope modes
#define SCM_IDLE		0
#define SCM_SINGLE		1
#define SCM_CONTINUOUS	2

// Triggering modes
#define SCTM_NONE		0
#define SCTM_NORMAL		1
#define SCTM_AUTO		2
#define SCTM_FREE		3
#define SCTM_SINGLE		4
#define SCTM_MANUAL		5

// Triggering conditions
#define SCTC_UP			1
#define SCTC_DOWN		2
#define SCTC_EDGE		3

// Stages
#define SCS_IDLE		0
#define SCS_PREVIEW		1
#define SCS_TRIGGER		2
#define SCS_FULL        3
#define SCS_TRIGGERED	4
#define SCS_COLLECT		5
#define SCS_END			6

// Types
#define TYPE_INT32              5
#define TYPE_UINT32             6
#define TYPE_FLOAT              7

struct ScopeConfigStruct {
	uint16_t Length;
	uint8_t Command, Mode, TrigMode, TrigCondition, nSignals, TrigType;
	uint16_t Depth, TrigPos;    // Output
	float Rate;                 // Output
    float TrigLevel, Span, Preview; 
	uint32_t Trig, Sig[8];
};

typedef float (*FloatConvert)(void* addr);
typedef int32_t (*IntConvert)(void* addr);

struct ScopeTrigStruct {
	float Level;				// Triggering level
	void* Var;					// Pointer to trigger variable
	FloatConvert Convert;		// Trigger variable conversion to float
	uint8_t Mode;				// 1 - normal, 2 - auto, 3 - free, 4 - single, 5 - manual
	uint8_t Condition;			// 1 - up, 2 - down, 3 - up-down
	int8_t Lt, Gt;				// flags Var <= Level, Var >= Level
	uint16_t pCount;			// Period counter
	int16_t sCount;				// Sample counter
};

struct ScopeSigStruct {
//	int32_t *bStore, *bRead;		// Pointers to current collection buffer and read buffer
	void* Var;					// Pointer to signal variable
	IntConvert Convert;			// Signal variable conversion to int32
};

struct ScopeSweepStruct {
	uint16_t Period; 				// in ticks
	uint16_t pCount;				// Period counter
	int16_t iStore;					// Indexes in collection and read buffers
	int16_t Depth;					// Requested number of samples
	int16_t Preview;				// Requested number of samples before triggering
	int16_t sCount;					// Number of collected samples
	uint8_t Full;					// Flag of buffer overrun
};

struct PacketStruct {
	uint8_t* Addr;
	uint16_t Count;
};
struct MultiPacketStruct {
    uint8_t Count;
    struct PacketStruct Packets[4];
};

struct ScopeStruct {
	struct ScopeConfigStruct Config;
	struct ScopeTrigStruct Trig;
	struct ScopeSweepStruct Sweep;
	struct ScopeSigStruct Sig[SC_MAXSIGNALS];
	uint8_t Stage, Mode;
    int8_t StoreBuf, SendBuf;				
	int32_t *pStore;		// Pointers to current collection buffer and read buffer};
    struct MultiPacketStruct Send[2];

    void Init();
    void Tick();
    uint8_t Configure(int32_t* from, uint16_t count);
    uint8_t Read(int32_t* buf);
    uint16_t FillSend(uint8_t buf);
    void Start(int8_t buf);
};

extern struct ScopeConfigStruct ScopeConfig;
extern struct ScopeStruct Scope;

