#include "chip.h"

#define FirmwareSector      ((uint32_t)2)    //Flash Sector 5 128 Kb
#define FirmwareAddress     0x08040000
#define FirmwareSize        (2 * 128 * 1024)
#define CacheSector      ((uint32_t)5)       //Flash Sector 8 128 Kb
#define CacheAddress     0x080A000
#define CacheSize        (2 * 128 * 1024)

#define FwLength ((uint32_t*)(FirmwareAddress+FirmwareSize-8))
#define FwChecksum ((uint32_t*)(FirmwareAddress+FirmwareSize-4))
#define FwIntvec ((uint32_t*)(FirmwareAddress))
#define CacheLength ((uint32_t*)(CacheAddress+CacheSize-8))
#define CacheChecksum ((uint32_t*)(CacheAddress+CacheSize-4))
#define CacheIntvec ((uint32_t*)(CacheAddress))

extern uint32_t __vector_table[100];

typedef  void (*pFunction)(void);

uint16_t uFlashUnlock() {
    if(FLASH->CR1 & 0x00000001) {   // If Flash locked, unlock it 
        FLASH->KEYR1 = ((uint32_t)0x45670123);
        FLASH->KEYR1 = ((uint32_t)0xCDEF89AB);
    }  
    FLASH->CCR1 = ((uint32_t)0x1FEF0000);   // Clear errors
    return 0;
}

uint16_t uFlashErase(uint32_t flashSector) {
    FLASH->CR1 = (uint32_t)0x00000004 | (flashSector << 8);     // Sector erase
    FLASH->CR1 |= (uint32_t)0x00000080;      // Start
    while (FLASH->SR1&(uint32_t)0x00000004) ;    // While Flash busy
    return FLASH->SR1 & (uint32_t)0x17EE0000;     // Error flags
}	

uint16_t  uFlashProgram(const uint32_t* flashAddr, const uint32_t* bufAddr, uint32_t len) {
    FLASH->CR1 = (uint32_t)0x00000002;           // Program
    for (int i = 0; i < len >> 2; i++) {
        *(uint32_t*)flashAddr++ = *bufAddr++;    
        while (FLASH->SR1 & (uint32_t)0x0000004) ;    // While Flash busy
        uint16_t r = (uint16_t)(FLASH->SR1 & (uint32_t)0x17EE0000);   // Error flags
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

int main(void) {
    // while (1);
    if ((FwIntvec[0] != 0xFFFFFFFF) && (FwIntvec[0] != 0)) {
        // Application is valid - jump to application
        __set_MSP(FwIntvec[0]);
        SCB->VTOR = (uint32_t)FwIntvec;
        ((pFunction)FwIntvec[1])();
    } else if ((*CacheLength != 0xFFFFFFFF) && (*CacheLength != 0) && (uCalculateCrc(CacheIntvec, *CacheLength) == *CacheChecksum)) {
        // Application is invalid, but the cache contains valid application copy 
        uint32_t len = *CacheLength;
        if (uFlashUnlock() != 0) while (1) ;
        if (uFlashErase(FirmwareSector) != 0) while (1) ;
        if (uFlashErase(FirmwareSector + 1) != 0) while (1) ;
        if (uFlashErase(FirmwareSector + 2) != 0) while(1) ;
        if (uFlashProgram(FwIntvec + 1, CacheIntvec + 1, len - 4) != 0) while (1) ;
        if (uFlashProgram(FwLength, CacheLength, 8) != 0) while(1) ;
        if (uFlashProgram(FwIntvec, CacheIntvec, 4) != 0) while (1) ;
        SCB->AIRCR = (SCB->AIRCR & 0x00000700) | 0x05FA0004;     // System reset
        __DSB();
        while (1);
    } else {
        // No valid application - wait forever
        while (1) ;
    }
    return 0;
}
