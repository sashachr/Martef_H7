// Global.cpp
// Global definitions, variables, routines
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"

#include <stdint.h>

#include "global.h"

uint16_t HardId;
uint16_t Initialization;

//volatile uint8_t ExecProgram;
//uint16_t ProgramValidated;

//#pragma DATA_SECTION("XMS");
byte Program[MaxProgram];
//#pragma DATA_SECTION(".ebss");


uint8_t Motor;      // 1=HR1, 2=HR2, and so on


#pragma location=0x08020200 
const GUID GuidDummy = { 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0 } };

uint32_t Properties[4] = {0, 0, 0, 0};
    // Bytes 0,1 - Size of flash page (for write)
    // Bytes 2,3 - Size of program block (for write)

void GetGuidUnit(uint32_t* dest) {
    *dest++ = 0x41423037;   // AB07
    uint32_t* uid = (uint32_t*)0x1FFF7A10;
    for (int i=0; i<3; i++) *dest++ = *uid++;
}

int GetGuidString(GUID* guid, uint8_t* buf, int len) {
    int n = len;
    if (n > 33) n = 33;
    uint8_t *g = (uint8_t*)guid, *b = buf;
    for (int i = 0; i < n; i++) {
        if (i == 32) {
            *b = 0;
        } else if ((i & 1) == 0) {
            *b++ = "0123456789ABCDEF"[*g >> 4];
        } else {
            *b++ = "0123456789ABCDEF"[*g++ & 0x0F];
        }
    }
    return n;
}

// Calculate Adler32 sum of the Buf, Count - byte length
uint32_t Adler32(int16_t* Buf, uint16_t Count) {
#define MOD_ADLER 65521
    uint32_t a = 1, b = 0;
    uint16_t index;
    for (index = 0; index < Count; ++index)
    {
    	uint32_t Byte = ((index&1)? Buf[index/2]>>8 : Buf[index/2])&0x00FF;
        a += Byte; if (a>=MOD_ADLER) a-=MOD_ADLER;
        b += a; if (b>=MOD_ADLER) b-=MOD_ADLER;
    }
    return (b << 16) | a;
}

// Calculate Fletcher32 sum of the Buf, Count is Buf length in 16-bit words
uint32_t Fletcher32(uint16_t* buf, uint16_t count) {
	uint32_t sum1 = 0x0000ffff, sum2 = 0x0000ffff;
    while (count) {
        uint16_t c = (count <= 359) ? count : 359;
        count -= c;
        while (c--) {
            sum1 += *buf++; sum2 += sum1;
        }
        sum1 = (sum1 & 0x0000ffff) + (sum1 >> 16);
        sum2 = (sum2 & 0x0000ffff) + (sum2 >> 16);
    }
    sum1 = (sum1 & 0x0000ffff) + (sum1 >> 16);
    sum2 = (sum2 & 0x0000ffff) + (sum2 >> 16);
	return (sum2 << 16) | sum1;
	// for (uint16_t i=0; i<count; i++) {
	// 	sum1 += *buf++; sum2 += sum1;
	// }
	// return ((uint32_t)sum2 << 16) | sum1;
}

// Restart firmware
void SysRestart() {
    SCB->AIRCR = (SCB->AIRCR & 0x00000700) | 0x05FA0004;     // System reset
    __DSB();
    while (1);
}

