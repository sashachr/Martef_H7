// Command.h
// Communication protocol
// COPYRIGHT 2012 Sasha Chrichov

#pragma once

//extern int32_t ConAddr;

struct SectionStruct {
	const uint8_t* Buf;
	uint16_t Bytes;
};

struct SendStruct {
	struct SectionStruct Sect[10];
	int16_t nSect, cSect;
    uint16_t* Flag;      // To zero after the end of transmit
};

enum MRE_ERRORS {
	MRE_ILLEGALCOMMAND = 101,
	MRE_FORMAT = 102,
	MRE_ILLEGALVARIABLE = 103,
	MRE_ILLEGALINDEX = 104,	
	MRE_READONLYVARIABLE = 105,	
	MRE_THREAD = 106,
	MRE_PROGRAMCHECKSUM = 107,
	MRE_PROGRAMMEMORYOVERFLOW = 108,
	MRE_PROGRAMHEADERINVALID = 109,
	MRE_FLASHERASEFAILED = 110,
	MRE_FLASHWRITEFAILED = 111,
	MRE_FLASHLOCKED = 112,
	MRE_FLASHCHECKSUM = 113,
	MRE_ILLEGALFUNCTION = 120,
	MRE_ILLEGALPARAMETERS = 121,
};

//extern struct SendStruct UartSendRequest;

void CommandInit();


void ExecuteCommand(uint8_t ConAddr, uint8_t* ComBuf, int ComCount, SendStruct* rep);
