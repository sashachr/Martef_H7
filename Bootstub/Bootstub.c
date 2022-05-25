#include "chip.h"

#define FirmwareSector      ((uint32_t)2)       // flash Sector 2
#define FirmwareAddress     0x08040000
#define FirmwareSize        (2 * 128 * 1024)
#define CacheSector         ((uint32_t)5)       // flash Sector 5
#define CacheAddress        0x080A0000
#define CacheSize           (2 * 128 * 1024)

#define FwChecksum ((uint32_t*)(FirmwareAddress))
#define FwLength ((uint32_t*)(FirmwareAddress + 4))
#define FwInvalidFlag ((uint32_t*)(FirmwareAddress + 0x0020))
#define FwProductGuid ((uint32_t*)(FirmwareAddress + 0x0080))
#define FwIntvec ((uint32_t*)(FirmwareAddress + 0x0100))
#define CacheChecksum ((uint32_t*)(CacheAddress))
#define CacheLength ((uint32_t*)(CacheAddress + 4))
#define CacheProductGuid ((uint32_t*)(CacheAddress + 0x0080))
#define CacheIntvec ((uint32_t*)(CacheAddress + 0x0100))

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
    FLASH->CCR1 = ((uint32_t)0x1FEF0000);   // Clear errors
    FLASH->CR1 = (uint32_t)0x00000024 | (flashSector << 8);     // Sector erase
    FLASH->CR1 |= (uint32_t)0x00000080;      // Start
    while (FLASH->SR1&(uint32_t)0x00000004) ;    // While Flash busy
    return FLASH->SR1 & (uint32_t)0x17EE0000;     // Error flags
}	

uint16_t  uFlashProgram(const uint32_t* flashAddr, const uint32_t* bufAddr, uint32_t len) {
    FLASH->CCR1 = ((uint32_t)0x1FEF0000);   // Clear errors
    FLASH->CR1 = (uint32_t)0x00000022;           // Program, word size
    for (int i = 0; i < len >> 2; i++) {
        *(uint32_t*)flashAddr++ = *bufAddr++;    
        while (FLASH->SR1 & (uint32_t)0x0000001) ;    // While Flash busy
        uint16_t r = (uint16_t)(FLASH->SR1 & (uint32_t)0x17EE0000);   // Error flags
        if (r) return r;    
    }
    if (FLASH->SR1 & (uint32_t)0x0000002) {
        FLASH->CR1 |= 0x00000040;     // Forced write
        while (FLASH->SR1 & (uint32_t)0x0000001) ;    // While Flash busy
        return (uint16_t)(FLASH->SR1 & (uint32_t)0x17EE0000);   // Error flags
    }
    return 0;
}

uint32_t uCalculateCrc(void* start, int bytes) {
    FLASH->CR1 = 0x00008000;            // Enable CRC
    FLASH->CRCCR1 = 0x000e0000;         // Clean CRC
    FLASH->CRCSADD1 = (uint32_t)start - 0x08000000;
    FLASH->CRCEADD1 = (uint32_t)start + bytes - 0x08000000 - 4;
    FLASH->CRCCR1 = 0x000d0000;         // Start CRC
    while (FLASH->SR1 & (uint32_t)0x0000001) ;    // While Flash busy
    uint32_t crc = FLASH->CRCDATA;
    FLASH->CR1 = 0x00000000;            // Disable CRC

    RCC->AHB4ENR |= 0x00080000;   // Enable CRC unit
    CRC->CR = 1;        // Reset
    while (CRC->CR == 1) ;
    uint32_t* w = (uint32_t*)start;
    uint32_t words = (bytes + 3) / 4;
    for (int i=0; i<words; i++) CRC->DR = *w++;
    uint32_t crc1 = CRC->DR;
    RCC->AHB4ENR &= ~0x00080000;   // Disable CRC unit
   
    return crc1;
}

int main(void) {
    if ((*FwLength == 0xFFFFFFFF) && (*FwIntvec != 0xFFFFFFFF) && (*FwInvalidFlag != 0x5A5A5A5A)) { 
        // After JTAG programming - initialize
        uint32_t last = FirmwareAddress + FirmwareSize;
        while (*(uint32_t*)(last - 4) == 0xFFFFFFFF) last -= 4;
        last = (last + 0x0000007F) & 0xFFFFFF80;        // Alignment to 128 bytes - necessary for CRC
        uint32_t len = last - (uint32_t)FwProductGuid;
        if (uFlashUnlock() != 0) while (1) ;
        uint32_t cs = uCalculateCrc(FwProductGuid, len);
        uint32_t w[] = {cs, len};
        if (uFlashProgram(FwChecksum, w, 8) != 0) while (1) ;
    }
    // uint32_t d = 0;
    // for (int i = 0; i < (*FwLength >> 2); i++) {
    // 	if (FwProductGuid[i] != CacheProductGuid[i])
    // 		d++;
    // }
    uint32_t fwvalid = (*FwIntvec != 0xFFFFFFFF) && (*FwInvalidFlag != 0x5A5A5A5A) && (*FwChecksum == uCalculateCrc(FwProductGuid, *FwLength));
    uint32_t cachevalid = (*CacheIntvec != 0xFFFFFFFF) && (*CacheChecksum == uCalculateCrc(CacheProductGuid, *CacheLength));
    if (fwvalid) {
        if (!cachevalid) {      // Copy to cache
            if (uFlashUnlock() != 0) while (1) ;
            if (uFlashErase(CacheSector) != 0) while (1) ;
            if (uFlashErase(CacheSector + 1) != 0) while (1) ;
            if (uFlashProgram(CacheChecksum, FwChecksum, *FwLength + 0x0020) != 0) while (1) ;
        }
    } else {
        if (cachevalid) {      // Copy to firmware
            if (uFlashUnlock() != 0) while (1) ;
            if (uFlashErase(FirmwareSector) != 0) while (1) ;
            if (uFlashErase(FirmwareSector + 1) != 0) while (1) ;
            if (uFlashProgram(FwChecksum, CacheChecksum, *CacheLength + 0x0020) != 0) while (1) ;
            fwvalid = (FwIntvec[0] != 0xFFFFFFFF) && (*FwChecksum == uCalculateCrc(FwProductGuid, *FwLength));
        }
    }
    if (!fwvalid) while (1) ; // No valid application - wait forever
    __set_MSP(FwIntvec[0]);
    SCB->VTOR = (uint32_t)FwIntvec;
    ((pFunction)FwIntvec[1])();
    return 0;
}
