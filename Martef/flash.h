#pragma once

extern uint8_t FlashOperationInProgress;

uint32_t FlashIsLocked();
uint32_t FlashLock();
uint32_t FlashUnlock();

uint32_t FlashEraseInitArea();
uint32_t FlashSaveInitArea();
uint32_t FlashGetInitArea();
uint32_t FlashUpgradeErase();
uint32_t FlashUpgradeWrite(uint32_t FlashOffs, uint32_t* Buf, uint32_t Len);
uint32_t FlashUpgradeInfo(uint32_t* info);
uint32_t FlashDiscardFirmware();
uint32_t FlashSaveFirmwareData(uint32_t length, uint32_t crc);
uint8_t FlashValidateFirmware();
uint8_t FlashValidateUpgrade();
uint32_t FlashSaveSerial(uint8_t* serial);
uint8_t* FlashGetUnitGuid();
uint8_t* FlashGetSerial();
int32_t FlashOperation(int32_t o);
