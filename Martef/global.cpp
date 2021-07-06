// Global.cpp
// Global definitions, variables, routines
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"

#include <stdint.h>

#include "Global.h"

uint16_t HardId;
uint16_t Initialization;

//volatile uint8_t ExecProgram;
//uint16_t ProgramValidated;

//#pragma DATA_SECTION("XMS");
byte Program[MaxProgram];
//#pragma DATA_SECTION(".ebss");

const uint8_t ProductString[] = "AB-07";
const uint8_t ManufacturerString[] = "Nanomotion";
uint8_t ApplicationString[60] = "";
// uint8_t const* MotorString = "Unknown";

uint8_t Motor;      // 1=HR1, 2=HR2, and so on

GUID GuidUnit = { 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0 } };

#pragma location=0x08020200 
const GUID __attribute__((used)) GuidProduct = { 0xc7d043ae, 0x7161, 0x4b5b, { 0xb3, 0xc0, 0x72, 0x1, 0xc8, 0x6d, 0x9c, 0x1 } }; // {C7D043AE-7161-4B5B-B3C0-7201C86D9C01}
const GUID GuidManufacturer = { 0xebb27f3b, 0x758f, 0x4dbe, { 0x94, 0x93, 0x4, 0x9f, 0xec, 0x2e, 0x8f, 0x22 } };  // {EBB27F3B-758F-4DBE-9493-049FEC2E8F22}
const GUID GuidFwVersion = { 0x113cc7c1, 0xa2d3, 0x46e1, { 0xad, 0x18, 0x27, 0xa2, 0x84, 0xf9, 0x86, 0xaa } };  // {113CC7C1-A2D3-46E1-AD18-27A284F986AA}
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

