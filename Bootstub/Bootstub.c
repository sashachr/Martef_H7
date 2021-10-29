#include "chip.h"

#define FirmwareSector      ((uint32_t)0x0028)    //Flash Sector 5 128 Kb
#define FirmwareAddress     0x08020000
#define FirmwareSize        (3 * 128 * 1024)
#define CacheSector      ((uint32_t)0x0040)       //Flash Sector 8 128 Kb
#define CacheAddress     0x08080000
#define CacheSize        (3 * 128 * 1024)

__root __no_init const uint32_t FwLength @ 0x0807fff8; 
__root __no_init const uint32_t FwChecksum @ 0x0807fffc;
__root __no_init const uint32_t CacheLength @ 0x080dfff8;
__root __no_init const uint32_t CacheChecksum @ 0x080dfffc;

__root __no_init const uint32_t FwIntvec[1] @ 0x08020000;
__root __no_init const uint32_t CacheIntvec[1] @ 0x08080000;

extern uint32_t __vector_table[100];

typedef  void (*pFunction)(void);

#pragma default_function_attributes = @ ".bootstub"

uint16_t uFlashUnlock() {
    if(FLASH->CR & 0x80000000) {   // If Flash locked, unlock it 
        FLASH->KEYR = ((uint32_t)0x45670123);
        FLASH->KEYR = ((uint32_t)0xCDEF89AB);
    }  
    FLASH->SR = ((uint32_t)0x000000F3);   // Clear flags
    return 0;
}

uint16_t uFlashErase(uint32_t flashSector) {
    FLASH->CR = (uint32_t)0x00000202 | flashSector;     // 32-bit word size, sector erase
    FLASH->CR |= (uint32_t)0x00010000;      // Start
    while (FLASH->SR&(uint32_t)0x00010000) ;    // While Flash busy
    return FLASH->SR & (uint32_t)0x000000F2;     // Error flags
}	

uint16_t  uFlashProgram(const uint32_t* flashAddr, const uint32_t* bufAddr, uint32_t len) {
    for (int i = 0; i < len >> 2; i++) {
        FLASH->CR = (uint32_t)0x00000201;           // 32-bit word size, program
        *(uint32_t*)flashAddr++ = *bufAddr++;    
        while (FLASH->SR & (uint32_t)0x00010000) ;    // While Flash busy
        uint16_t r = (uint16_t)(FLASH->SR & (uint32_t)0x000000F2);   // Error flags
        if (r) return r;    
    }
    return 0;
}

uint32_t uCalculateCrc(void* start, int bytes) {
    RCC->AHB1ENR |= 0x00001000;   // Enable CRC unit
    uint32_t* w = (uint32_t*)start;
    uint32_t words = (bytes + 3) / 4;
    for (int i=0; i<words; i++) CRC->DR = *w++;
    uint32_t crc = CRC->DR;
    RCC->AHB1ENR &= ~0x00001000;   // Disable CRC unit
    return crc;
}

void BootStub() {
    // while (1);
    if ((FwIntvec[0] != 0xFFFFFFFF) && (FwIntvec[0] != 0)) {
        // Application is valid - jump to application
        __set_MSP(FwIntvec[0]);
        SCB->VTOR = (uint32_t)FwIntvec;
        ((pFunction)FwIntvec[1])();
    } else if ((CacheLength != 0xFFFFFFFF) && (CacheLength != 0) && (uCalculateCrc((void*)CacheAddress, CacheLength) == CacheChecksum)) {
        // Application is invalid, but the cache contains valid application copy 
        uint32_t len = CacheLength;
        if (uFlashUnlock() != 0) while (1) ;
        if (uFlashErase(FirmwareSector) != 0) while (1) ;
        if (len > 128 * 1024) { if (uFlashErase(FirmwareSector + 0x08) != 0) while (1) ;}
        if (uFlashErase(FirmwareSector + 0x10) != 0) while(1) ;
        if (uFlashProgram(FwIntvec + 1, CacheIntvec + 1, len - 4) != 0) while (1) ;
        if (uFlashProgram(&FwLength, (uint32_t*)&CacheLength, 8) != 0) while(1) ;
        if (uFlashProgram(FwIntvec, CacheIntvec, 4) != 0) while (1) ;
        SCB->AIRCR = (SCB->AIRCR & 0x00000700) | 0x05FA0004;     // System reset
        __DSB();
        while (1);
     } else {
        // No valid application - wait forever
        while (1) ;
    }
}