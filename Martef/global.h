// Global.cpp
// Global definitions, variables, routines
// COPYRIGHT 2012 Sasha Chrichov

#pragma once

typedef uint8_t  byte;

#define NAX        6               // Number of axes
#define N_ENCODERS    2               // Number of Encoders
#define N_PWMS		  2				  // Number of PWMs
#define N_CHANNELS    3               // Number of communication channels


#define SERVO_20KHZ
#define AX_2

#define SYSCLK_RATE				550000000L		// 550 MHz
#define APB1_RATE   			137500000L   	// 137.5 MHz
#define APB2_RATE   			137500000L   	// 137.5 MHz
#define APB4_RATE   			137500000L   	// 137.5 MHz
#define APB4_RATE   			137500000L   	// 275 MHz

#ifdef SERVO_20KHZ
  #define TICKS_IN_SECOND		20000.0F
  #define TICKS_IN_MILLISECOND	20.0F
  #define SECONDS_IN_TICK		0.00005F
  #define MILLISECONDS_IN_TICK	0.05F
#endif
#ifdef SERVO_10KHZ
  #define TICKS_IN_SECOND		10000.0
  #define TICKS_IN_MILLISECOND	10.0
  #define SECONDS_IN_TICK		0.0001
  #define MILLISECONDS_IN_TICK	0.1
#endif

#define N_GPIO	8
#define N_AIN	16
#define N_AOUT	2

#define F_PI  3.1415926535F

//#define TIMER_DIVIDER	((int)(CPU_CLOCK/TICKS_IN_SECOND+0.5))
//#define TIMER_FACTOR	(100.F/(float)TIMER_DIVIDER)
#define RS232_COMMAND_POLL	((int)(0.1*TICKS_IN_MILLISECOND+0.5))		// Poll RS232 command
#define MULTI_STEP_TICKS_IN_MILLISECOND ((int)(TICKS_IN_MILLISECOND/RS232_COMMAND_POLL)) 

#define StackSize			40

#define MaxProgram			2048		// Maximal program length in words 
#define ProgramChunk		40			// Length of program communication block in bytes

#define MaxInit				8*1024		// Length of initialization records area in words
#define FlashChunk			40			// Length of flash communication block in bytes

inline uint8_t Hex2Nibble(uint8_t hex) { return (hex <= '9') ? hex - '0' : (hex <= 'F') ? hex - 'A' + 10 : hex - 'a' + 10; }

struct GUID {
  uint32_t Data1;
  uint16_t Data2;
  uint16_t Data3;
  uint8_t  Data4[8];
};

extern const uint8_t ProductString[];
extern const uint8_t ManufacturerString[];
extern const uint8_t GitVersion[];
extern uint8_t SerialNumberString[60];
extern uint8_t ApplicationString[60];
extern const uint8_t CdfString[];
extern const uint8_t emptystring[1];
extern const struct GUID ProductGuid;
extern const struct GUID ManufacturerGuid;
extern struct GUID UnitGuid;
extern const struct GUID VersionGuid;
//extern struct GUID GuidFwVersion;
extern const struct GUID GuidDummy;

extern uint16_t HardId;
extern uint16_t Initialization;
//extern volatile uint8_t ExecProgram;
//extern uint16_t ProgramValidated;

extern byte Program[MaxProgram];

// Execution/Error codes
#define PROG_NOTEXECUTED			    0
#define PROG_EXECUTING				    1
#define PROG_FINISHED				      2
#define PROG_TERMINATED				    3
#define PROG_LOGPOINT				      4
#define PROG_BREAKPOINT				    5
#define PROG_PAUSED				        6
#define FLT_UNKNOWN 				      1000
#define FLT_POSLIMIT              1001
#define FLT_POSERROR				      1002
#define FLT_SOFTLIMIT				      1003
#define FLT_OVERCURRENT			      1010
#define FLT_POWERVOLTAGE		      1011
#define FLT_GATEUVLO              1020
#define FLT_GATETHERMAL           1021
#define FLT_GATEVDS               1022
#define FLT_GATERESET             1023
#define FLT_GATE                  1024
#define FLT_GATEI2C               1025
#define FLT_INDUCED               1050
#define FLT_MOTIONTIMEOUT         1030
#define RTE_ILLEGALCOMMAND		    201
#define RTE_NONWAITFAILURE		    202
#define RTE_FUNCTIONNOTSUPPORTED	203
#define RTE_MATHERROR				      204
#define RTE_ILLEGALVALUE			    205
#define RTE_MEMORYOVERFLOW			  206
#define RTE_THREADOVERFLOW			  207
#define MSE_INVALID				        300
#define MSE_METHOD				        301
#define MSE_AXIS				          302
#define MSE_TIMEOUT				        315
#define MSE_FAILURE				        320
#define MSE_NOCOMMUTATION				  330
#define TST_NOMOTION			        401
#define TST_LOWVELOCITY			      402
#define TST_DIRECTION			        403
#define TST_SLOWMOTION		        404
#define TST_INDEX				          405
#define TST_BRICKTIME			        406
// Immediate command errors
#define MRE_WRONGID               100
#define MRE_WRONGINDEX            101
#define MRE_WRONGVALUE            102
#define MRE_NOCOMMUT              103

extern const uint8_t ProductString[];
extern const uint8_t ManufacturerString[];
// extern const uint8_t GitVersion[];
// extern const uint8_t GitSha[];
extern uint8_t ApplicationString[];
extern uint8_t const* MotorString;

extern const uint16_t Application;
extern const uint32_t Version;
extern const uint32_t Serial;

// extern uint32_t Properties[];
// #define FLASH_PAGE (*(uint16_t*)Properties)
// #define PROGRAM_BLOCK 256

// extern uint8_t Motor;      // 1/2/4/6/8/12/16/24=HR1/2/4/6/8/12/16/24, 101/102/104/106/108/112/116/124=SE1/2/4/6/8/12/16/24

#if defined(USE_DCACHE) 
    #define InvalidateDCacheIfUsed(addr, size) SCB_InvalidateDCache_by_Addr(addr, size) 
    #define CleanDCacheIfUsed(addr, size) SCB_CleanDCache_by_Addr(addr, size)
    #define CleanInvalidateDCacheIfUsed(addr, size) SCB_CleanInvalidateDCache_by_Addr(addr, size)
#else
    #define InvalidateDCacheIfUsed(addr, size) 
    #define CleanDCacheIfUsed(addr, size) 
    #define CleanInvalidateDCacheIfUsed(addr, size) 
#endif


void GetUnitGuid(uint32_t* dest);
inline void GuidFromString(struct GUID* guid, const uint8_t* str) {
    uint8_t* g = (uint8_t*)guid;
    for (int i = 0; i < 32; i++) {
        uint8_t nibble = *str++;
        uint8_t v = (nibble < 0x3A) ? nibble - 0x30 : (nibble < 0x47) ? nibble - 0x41 + 10 : nibble - 0x61 + 10;
        if (i & 1) *g++ |= v; else *g = v << 4; 
    }    
}

// Both source and dest should be 32-bit aligned
inline void MemCpy32(void* dest, void* source, uint32_t count) { 
  	uint32_t *d = (uint32_t*)dest, *s = (uint32_t*)source;
  	for (uint32_t i = 0; i < count; i++) *d++ = *s++; 
} 
inline int Min(int a, int b) { return (a <= b) ? a : b; }
inline float Minf(float a, float b) { return (a <= b) ? a : b; }

inline uint16_t IsNan(float V)
{
	return ((((*(long*)&V)&0x7F800000L)==0x7F800000L) && (((*(long*)&V)&0x007FFFFFL)!=0));
}

inline float GetNan()
{
	long N = 0x7F800001L;
	return *(float*)&N;
}

union Any32 {float Float; int32_t Int; uint32_t Uint;};

inline float GetNone() { union Any32 n; n.Uint = 0xFFFFFFFF; return n.Float; }
#define NONE GetNone()

uint32_t Adler32(int16_t* Buf, uint16_t Count);

// Calculate Fletcher32 sum of the Buf, Count is Buf length in 16-bit words
uint32_t Fletcher32(uint16_t* buf, uint16_t count);

int GetGuidString(struct GUID* guid, uint8_t* buf, int len);

void SysRestart();

