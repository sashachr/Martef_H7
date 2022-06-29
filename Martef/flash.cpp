/**
  ******************************************************************************
  * @file    Flash.c 
  * @author  Sasha
  * @version Vx.x.x
  * @date    xx-xx-xx
  * @brief   Main program body
  ******************************************************************************
  * <h2><center>&copy; COPYRIGHT 2013 Sasha</center></h2>
  ******************************************************************************
  */ 

#include "chip.h"
#include <string.h>

#include "global.h"
#include "sysvar.h"
#include "flash.h"

// Flash Memory Map
// Sector  Start       End         Size    Remark
// 0       0x08000000  0x08003FFF  16K     Bootstub
// 2       0x08008000  0x0800BFFF  16K     Serial number
// 3       0x0800C000  0x0800FFFF  16K     Parameters save
// 5,6,7   0x08020000  0x0807FFFF  3*128K  Firmware
// 8,9,10  0x08080000  0x080DFFFF  3*128K  Firmware cache

#define InitSector          ((uint32_t)1)          // Sector 1 Init area 128 Kb
#define InitAddr            ((uint32_t)0x08020000) // Base address of Sector 1, 16 Kbytes
#define InitStart           ((uint32_t)0x08020080) // Init data starts here
#define InitSpace           (128 * 1024)           // Size of Init area
#define FirmwareSector      ((uint32_t)2)          // Sector 2 Firmware
#define FirmwareAddr        ((uint32_t)0x08040000) // Base address of Sector 2, 128 Kbytes
#define FirmwareStart       ((uint32_t)0x08040080) // Firmware first byte
#define FirmwareSpace       (2 * 128 * 1024)       // Size of Firmware area
#define UpgradeCacheSector  ((uint32_t)5)          // Sector 5 Firmware cache
#define UpgradeCacheAddr    ((uint32_t)0x080A0000) // Base address of Sector 5, 128 Kbytes
#define UpgradeCacheStart   ((uint32_t)0x080A0080) // Cache firmware first byte
#define UpgradeCacheSpace   (2 * 128 * 1024)       // Size of Firmware cache

uint8_t FlashOperationInProgress;
int InitDataValid;

uint32_t FlashIsLocked() { return (FLASH->CR1 & 0x00000001) != 0; }
uint32_t FlashLock() { if ((FLASH->CR1 & 0x00000001) == 0) FLASH->CR1 = 0x00000001; return 0; }
uint32_t FlashUnlock() {
    if (FLASH->CR1 & 0x00000001) {   // If Flash locked, unlock it 
        FLASH->KEYR1 = ((uint32_t)0x45670123);
        FLASH->KEYR1 = ((uint32_t)0xCDEF89AB);
    }  
    FLASH->CCR1 = ((uint32_t)0x1FEF0000);   // Clear flags
    return 0;
}

uint32_t flashErase(uint32_t flashSector) {
    FLASH->CCR1 = ((uint32_t)0x1FEF0000);   // Clear errors
    FLASH->CR1 = (uint32_t)0x00000024 | (flashSector << 8);     // Sector erase
    FLASH->CR1 |= (uint32_t)0x00000080;      // Start
    while (FLASH->SR1&(uint32_t)0x00000004) ;    // While Flash busy
    return FLASH->SR1 & (uint32_t)0x17EE0000;     // Error flags
}	

uint32_t flashProgramStart() {
    FLASH->CCR1 = ((uint32_t)0x1FEF0000);   // Clear errors
    FLASH->CR1 = (uint32_t)0x00000022;           // Program, word size
    return FLASH->SR1 & (uint32_t)0x17EE0000;   // Error flags
}
uint32_t flashProgramWord(uint32_t *FlashAddr, uint32_t *BufAddr) {
    *FlashAddr = *BufAddr;    
    while (FLASH->SR1 & (uint32_t)0x0000001) ;  // While Flash busy
    return FLASH->SR1 & (uint32_t)0x17EE0000;   // Error flags
}
uint32_t flashProgramEnd() {
    if (FLASH->SR1 & (uint32_t)0x0000002) {
        FLASH->CR1 |= 0x00000040;     // Forced write
        while (FLASH->SR1 & (uint32_t)0x0000001) ;    // While Flash busy
    }
    return FLASH->SR1 & (uint32_t)0x17EE0000;   // Error flags
}

uint32_t  flashProgram(uint32_t FlashAddr, uint32_t *BufAddr, uint32_t Length) {
    FLASH->CCR1 = ((uint32_t)0x1FEF0000);   // Clear errors
    FLASH->CR1 = (uint32_t)0x00000022;           // Program, word size
    for (int i = 0; i < Length >> 2; i++, FlashAddr += 4, BufAddr++) {
        *(uint32_t*)FlashAddr = *BufAddr;    
        while (FLASH->SR1 & (uint32_t)0x0000001) ;    // While Flash busy
        uint32_t r = (uint16_t)(FLASH->SR1 & (uint32_t)0x17EE0000);   // Error flags
        if (r) return r;    
    }
    if (FLASH->SR1 & (uint32_t)0x0000002) {
        FLASH->CR1 |= 0x00000040;     // Forced write
        while (FLASH->SR1 & (uint32_t)0x0000001) ;    // While Flash busy
    }
    return 0;
}

#define F_OP(o) {uint16_t r; if ((r = (o)) != 0) {FlashOperationInProgress = 0; return r;}}
#define F_PROGRAM(a, v, s) F_OP(flashProgram(a, v, s)); a += s

uint16_t FlashReadInitArea() {
    // uint32_t* f = (uint32_t*)(InitAddr + offs);
    // for (int i = 0; i < (len >> 2); i++) *buf++ = *f++;
  	return 0;
}

uint32_t FlashUpgradeErase() {
    uint32_t ret = flashErase(UpgradeCacheSector);
    if (ret == 0) ret = flashErase(UpgradeCacheSector + 1);
    return ret;
}

uint32_t FlashUpgradeWrite(uint32_t offs, uint32_t* buf, uint32_t len) {
    return flashProgram(UpgradeCacheAddr + offs, buf, len);
}

// uint32_t FlashUpgradeInfo(uint32_t* info) {
//     return flashProgram(UpgradeCacheAddr + UpgradeCacheSpace - 8, info, 8);
// }

uint32_t FlashDiscardFirmware() {
    uint32_t invalidflag = 0x5a5a5a5a;
    return flashProgram(FirmwareAddr + 0x0020, &invalidflag, 4);
}

uint32_t FlashCalculateCrc(void* start, int bytes) {
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

uint8_t FlashValidateFirmware() {
    return FlashCalculateCrc((uint32_t*)FirmwareStart, *((uint32_t*)FirmwareAddr + 1)) != *(uint32_t*)FirmwareAddr;
}

uint8_t FlashValidateUpgrade() {
    return FlashCalculateCrc((uint32_t*)UpgradeCacheStart, *((uint32_t*)UpgradeCacheAddr + 1)) != *(uint32_t*)UpgradeCacheAddr;
}

uint32_t FlashSaveInitArea() {
    if (FlashOperationInProgress) return 1;
    FlashOperationInProgress = 1;	
    F_OP(flashErase(InitSector));
    F_OP(flashProgramStart());
    uint32_t* a = (uint32_t*)InitStart;
    for (int i=0; i<nSysVars; i++) {
        if ((SysVars[i].size == 0) || !(SysVars[i].flags & VF_FLASH)) continue;
        uint32_t v = i | (SysVars[i].size << 16);
        F_OP(flashProgramWord(a++, &v)); 
        for (int j=0; j<SysVars[i].size; j++) {
            SysVars[i].read(j, 1, (int32_t*)&v);
            F_OP(flashProgramWord(a++, &v)); 
        }
    }
    F_OP(flashProgramEnd());
//    F_OP(flashValidateArea(InitAddr, InitSpace));
    FlashOperationInProgress = 0;
    return 0;
}

uint32_t FlashEraseInitArea() {
    if (FlashOperationInProgress) return 1;
    FlashOperationInProgress = 1;	
    F_OP(flashErase(InitSector));
    FlashOperationInProgress = 0;
    return 0;
}

uint32_t FlashGetInitArea() {
//    F_OP(flashValidateArea(InitAddr, InitSpace));
    uint32_t bytes = *(uint32_t*)(InitAddr + 4);
    if (bytes >= InitSpace) return -1;
    uint32_t *p = (uint32_t*)InitStart, *end = (uint32_t*)(InitStart + bytes);
    while (p < end) {
        uint32_t head = *p++;
        uint16_t id = head & 0x0000FFFF, count = head >> 16;
        if ((id < nSysVars) && (SysVars[id].size > 0) && (SysVars[id].flags & VF_FLASH)) {
            if (count > SysVars[id].size) count = SysVars[id].size;
            SysVars[id].write(0, count, (int32_t*)p);
        }
    }
    return 0;
}

uint32_t FlashSaveSerial(uint8_t* serial) {
    // uint32_t len = strlen((char const*)serial);
    // if (len > 79) return 1;
    // uint8_t res = len & 0x00000003;     // Alignment
    // len &= 0xFFFFFFFC;
    // uint32_t last = 0;
    // for (int i=0; i<res; i++) *((uint8_t*)&last + i) = *(serial + len + i); 
    // uint32_t a = OtpAddr;
    // uint32_t* s = (uint32_t*)a;
    // for (int i=0; i<20; i++) if (*s++ != 0xFFFFFFFF) return 1;  // Serial Number can be programed only once
    // F_PROGRAM(a, (uint32_t*)serial, len);
    // F_PROGRAM(a, &last, 4);
	return 0;
}
uint8_t NoSerial;
uint8_t* FlashGetSerial() {
//   uint8_t i = 0;
//   uint32_t* s = (uint32_t*)OtpAddr;
//   for (; i<20; i++) if (*s++ != 0xFFFFFFFF) break;  
//   return (i == 20) ? &NoSerial : (uint8_t*)(OtpAddr);
    return 0;
}
int32_t FlashOperation(int32_t o) {
    switch (o) {
        case 1: return (FlashSaveInitArea() != 0) ? 0 : 1;
        case 2: return (FlashEraseInitArea() != 0) ? 0 : 1;
        default: return 0;
    }
}


