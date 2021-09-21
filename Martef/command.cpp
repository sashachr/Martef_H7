// Command.cpp
// Communication protocol
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"
#include <string.h>

#include "global.h"
#include "sysvar.h"
//#include "martel.h"
#include "gitversion.h"
#include "communication.h"
#include "command.h"

//  FD command structure:
//	Byte 1: FD
//	Byte 2: Command
//	Byte 3,4: ID
//	Byte 4,5: Length

// Commands 
enum MRC_COMMAND {
	MRC_READVAR = 201,
	MRC_READARRAY = 202,
	MRC_WRITEVAR = 203,
	MRC_WRITEARRAY = 204,
	MRC_READMULTI = 205,
	MRC_WRITEMULTI = 206,
	MRC_READGUID = 207,
	MRC_WRITEGUID = 208,
	MRC_READSTRING = 209,
	MRC_WRITESTRING = 210,
	MRC_FUNCTION = 240,
	MRC_UPGRADE = 250,
};

#define RamD2HeapSize  4096
__attribute__((section(".ramD2"))) uint32_t RamD2Heap[RamD2HeapSize];
uint32_t* RamD2HeapFree = RamD2Heap;

uint32_t* AllocRamD2(uint32_t size) {
	size = (size + 3) >> 2;		// number of 32-bit words
	if (((RamD2HeapSize >> 2) - (RamD2HeapFree - RamD2Heap)) < size) return 0;
	uint32_t* r = RamD2HeapFree;
	RamD2HeapFree += size;
	return r;
}

#define nstrings 7
uint8_t* DStrings[nstrings];
uint16_t DStringsLen[nstrings];
SaDmaStruct dmaStrings;

__attribute__((section(".mdmalink"))) static struct MdmaLink mlReply[4];
__attribute__((section(".ramD1init"))) uint8_t ProductString[] = "Martef-H7";
__attribute__((section(".ramD1init"))) uint8_t ManufacturerString[] = "XACT Robotics";
__attribute__((section(".ramD1"))) uint8_t SerialNumberString[60];
__attribute__((section(".ramD1"))) uint8_t FwVersionString[60];
__attribute__((section(".ramD1"))) uint8_t ApplicationString[60];
__attribute__((section(".ramD2init"))) uint8_t CdfString[] = 
	#include "xact.cdc"
	"\x00\x00\x00"	// for alignment
;
__attribute__((section(".ramD1init"))) static uint8_t emptystring[1] = { 0 };
#define nguids 5
__attribute__((section(".ramD1init"))) GUID GuidProduct = { 0xc7d043ae, 0x7161, 0x4b5b, { 0xb3, 0xc0, 0x72, 0x1, 0xc8, 0x6d, 0x9c, 0x1 } }; // {C7D043AE-7161-4B5B-B3C0-7201C86D9C01}
__attribute__((section(".ramD1init"))) GUID GuidManufacturer = { 0xebb27f3b, 0x758f, 0x4dbe, { 0x94, 0x93, 0x4, 0x9f, 0xec, 0x2e, 0x8f, 0x22 } };  // {EBB27F3B-758F-4DBE-9493-049FEC2E8F22}
__attribute__((section(".ramD1"))) GUID GuidUnit;
__attribute__((section(".ramD1init"))) GUID GuidFwVersion { 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0 } };
__attribute__((section(".ramD1init"))) static GUID emptyguid { 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0 } };
__attribute__((section(".mdmalink"))) static struct MdmaLink mlGuids[nguids];

#define BREAK(e) {out[1] = e; break;}

void FdProtocol(uint8_t* buf, uint16_t count, CommChannel* ch) {
	uint16_t id = *(uint16_t*)&buf[2], clen = *(uint16_t*)&buf[4];
	uint8_t* out = ch->Outbuf;
	out[0] = 0xFD; out[1] = 0;		// Success assumed
	*(uint16_t*)(out + 2) = id;
	*(uint16_t*)(out + 4) = 8;		// Default prompt assumed
	uint16_t outlen = 8;			// Reply length or partial reply length in case of SaDma
	*(uint16_t*)(out + 6) = 0;		// Default prompt assumed
	switch (buf[1]) {
		case MRC_READVAR: {
			// if (clen != 8) BREAK(MRE_FORMAT)
			// uint16_t var = *(uint16_t*)(buf + 6);
            // *(uint16_t*)(out + 6) = var;
            // if (var < 10000) {      // System variable
            //     if ((var >= nSysVars) || (SysVars[var].Size <= 0)) BREAK(MRE_ILLEGALVARIABLE)
            //     *(uint16_t*)(out + 6) = var;
            //     int n = SysVars[var].Size;
            //     if (n > 16) n = 16;
            //     *(uint16_t*)(out + 4) = rep->Sect[0].Bytes = 8 + (n << 2);
            // } else if (var < 20000) { // User scalar
            //     var -= 10000;
            //     *(uint16_t*)(out + 4) = rep->Sect[0].Bytes = 12;
            // } else if (var < 30000) { // User array
            //     var -= 20000;
            //     int32_t* ar = 0; // (int32_t*)MrtInfo.Arrays[var];
            //     int size = *ar++;
            //     if (size <= 16) {
            //         for (uint16_t i = 0; i < size; i++) *(uint32_t*)(out + 8 + (i << 2)) = *ar++;
            //         *(uint16_t*)(out + 4) = rep->Sect[0].Bytes = 8 + (size << 2);
            //     } else {
            //         rep->nSect = 2;
            //         *(uint16_t*)(out + 4) = 8 + (size << 2);
            //         rep->Sect[0].Bytes = 8;
            //         rep->Sect[1].Buf = (uint8_t*)(ar + 1);
            //         rep->Sect[1].Bytes = size << 2;
            //     }
            // }	
			break;
		}
		case MRC_READARRAY: {
			if (clen != 12) BREAK(MRE_FORMAT)
			uint16_t var = *(uint16_t*)(buf+6), ind = *(uint16_t*)(buf+8), count = *(uint16_t*)(buf+10);
			*(uint16_t*)(out + 6) = var;
            *(uint16_t*)(out + 8) = ind;
            if (var < 10000) {      // System variable
                if (var >= nSysVars) BREAK(MRE_ILLEGALVARIABLE)
				Vardef* vd = SysVars + var;
                int n = vd->size;
				if (n == 0) BREAK(MRE_ILLEGALVARIABLE)
                if (ind + count > n) BREAK(MRE_ILLEGALINDEX)
				int32_t acount = vd->read(ind, count, (int32_t*)(out+12));
				if (acount >= 0) {
		            *(uint16_t*)(out + 10) = acount;
					outlen = *(uint16_t*)(out + 4) = 12 + (acount << 2);
				} else {
					ch->SaDma = (SaDmaStruct*)*(uint32_t*)(out+12);
					uint16_t len = 0;
					for (int i = 0; i < ch->SaDma->pCount; i++) len += ch->SaDma->Packets[i].Count;
		            *(uint16_t*)(out + 10) = len >> 2;
		            *(uint16_t*)(out + 4) = 12 + len;
		            outlen = 12;
				}
			} else if (var < 20000) {   // User scalar
                // if (ind + count > 1) BREAK(MRE_ILLEGALINDEX)
                // *(int32_t*)&out[12] = MrtInfo.Globals[var-10000];
                // *(uint16_t*)(out + 4) = 16;
			} else if (var < 30000) {   // User array
                // int32_t *a = (int32_t*)MrtInfo.Arrays[var-20000], n = *a;
                // if (ind + count > n) BREAK(MRE_ILLEGALINDEX)
                // rep->nSect = 2;
                // rep->Sect[0].Bytes = 12;
                // rep->Sect[1].Buf = (uint8_t*)(a + ind);
                // rep->Sect[1].Bytes = count << 2;
            }
		    CleanDCacheIfUsed((uint32_t*)out, outlen);
			break;
		}
		case MRC_WRITEARRAY: {
			if (clen < 12) BREAK(MRE_FORMAT)
			uint16_t var = *(uint16_t*)(buf+6), ind = *(uint16_t*)(buf+8), count = *(uint16_t*)(buf+10);
			if (clen != 12 + count * 4) BREAK(MRE_FORMAT)
            if (var < 10000) {      // System variable
                if (var >= nSysVars) BREAK(MRE_ILLEGALVARIABLE)
				Vardef* vd = SysVars + var;
                int n = vd->size;
				if (n == 0) BREAK(MRE_ILLEGALVARIABLE)
                if (ind + count > n) BREAK(MRE_ILLEGALINDEX)
				if (vd->write == 0) BREAK(MRE_READONLYVARIABLE)
				vd->write(ind, count, (int32_t*)(buf+12));
			} else if (var < 20000) {   // User array
                // int32_t v = var - 10000;
                // if (v >= MrtInfo.nGlobals) BREAK(MRE_ILLEGALVARIABLE)
                // if (ind + count > 1) BREAK(MRE_ILLEGALINDEX)
                // MrtInfo.Globals[v] = *(int32_t*)(buf+12);
			} else if (var < 30000) {   // User array
                // int32_t v = var - 20000;
                // if (v >= MrtInfo.nArrays) BREAK(MRE_ILLEGALVARIABLE)
                // int32_t *a = (int32_t*)MrtInfo.Arrays[v], n = *a;
                // if (ind + count > n) BREAK(MRE_ILLEGALINDEX)
				// MemCpy32(a + ind + 1, (int32_t*)(buf + 12), count); 
			}
		    CleanDCacheIfUsed((uint32_t*)out, 8);
			break;
		}
		case MRC_READMULTI: {
			if (clen < 12) BREAK(MRE_FORMAT)
			uint16_t* ip = (uint16_t*)(buf+6);
			uint16_t count = *ip++;
			if (count > 64) BREAK(MRE_FORMAT)
			if (clen != 8 + 4 * count) BREAK(MRE_FORMAT)
			int32_t* op = (int32_t*)(out+8);
			for (int i = 0; i < count; i++) {
				uint16_t var = *ip++, ind = *ip++;
                if (var < 10000) {      // System variable
	    			if ((var >= nSysVars) || (SysVars[var].size == 0)) {
						*op++ = -1;
					} else {
						SysVars[var].read(ind, 1, op++);
					} 
    			} else if (var < 20000) {   // User scalar
                    // uint16_t v = var - 10000;
                    // *op++ = ((v < MrtInfo.nGlobals) && (ind == 0)) ? MrtInfo.Globals[v] : -1;                
    			} else if (var < 30000) {   // User array
                    // uint16_t v = var - 20000;
                    // int32_t* a = (v < MrtInfo.nArrays) ? (int32_t*)MrtInfo.Arrays[v] : 0;
                    // *op++ = ((a > 0) && (ind < *a)) ? a[ind+1] : -1;                
                } else {
                    *op++ = -1;                
                }
			}	
			*(uint16_t*)(out+6) = count;
			outlen = *(uint16_t*)(out+4) = 8 + (count << 2);
		    CleanDCacheIfUsed((uint32_t*)out, outlen);
			break;
		}
		case MRC_READSTRING: {
			if (clen != 12) BREAK(MRE_FORMAT)
			uint16_t ind = *(uint16_t*)&buf[8], count = *(uint16_t*)&buf[10];
			if (ind >= nstrings) BREAK(MRE_WRONGINDEX)
			int16_t rem = nstrings - ind; 
			if (count > rem) count = rem;
            outlen = 0;
			dmaStrings.pCount = dmaStrings.pCounter = 0;
			PacketStruct* p = dmaStrings.Packets;
			if (ind < 6) {
				p->Addr = DStrings[ind];
				for (int i = 0; i < ((count < 6) ? count : 6); i++) outlen += DStringsLen[i];
				p->Count = outlen;
				dmaStrings.pCount++; p++;
			}
			if (ind+count == 7) {
				p->Addr = DStrings[6];
				p->Count = DStringsLen[6];
				outlen += p->Count;
				uint16_t align = -outlen & 0x0003;	// alignment
				p->Count += align; outlen += align;
				dmaStrings.pCount++; 
			}
			ch->SaDma = &dmaStrings;
            *(uint16_t*)(out + 4) = 12 + outlen;
            *(uint16_t*)(out + 6) = 0;
            *(uint16_t*)(out + 8) = ind;
            *(uint16_t*)(out + 10) = count;
		    CleanDCacheIfUsed((uint32_t*)out, 12);
			outlen = 12;
            break;
        }
		case MRC_WRITESTRING: {
			// if (clen < 12) BREAK(MRE_FORMAT)
			// uint16_t ind = *(uint16_t*)&buf[8], count = *(uint16_t*)&buf[10];
			// if ((ind != 2) || (count != 1)) BREAK(MRE_ILLEGALINDEX)
            // if (FlashIsLocked()) BREAK(MRE_FLASHLOCKED)
			// *(uint16_t*)&out[6] = 0; *(uint16_t*)&out[8] = 2; *(uint16_t*)&out[10] = 1;
            // uint16_t r = FlashSaveSerial(&buf[12]);
            // if (r != 0) BREAK(MRE_FLASHWRITEFAILED)
			// rep->Sect[0].Bytes = 12;
			break;
		}
		case MRC_READGUID: {
			if (clen != 12) BREAK(MRE_FORMAT)
			uint16_t ind = *(uint16_t*)&buf[8], count = *(uint16_t*)&buf[10];
			*(uint16_t*)(out + 8) = ind; *(uint16_t*)(out + 10) = count;
            outlen = 12;
			struct MdmaLink* ml = (ind < nguids) ? mlGuids + ind : 0;
			int i = 0, ii = ind;
            for ( ; (i < count) && (ii < nguids); i++, ii++, ml++) {
				ml->CDAR = (uint32_t)(out + outlen);
				ml->CLAR = (uint32_t)(ml + 1);
				outlen += 16;
			} 
			int fill = count - i;  // overindexing
			if (fill) {
				if (ml != 0) (ml-1)->CLAR = (uint32_t)(mlReply);
				Mdma::InitLink(mlReply[0], 0x70000008, fill << 4, (uint32_t)emptystring, (uint32_t)(out + outlen));
				outlen += fill << 4; 
			} else {
				(ml-1)->CLAR = 0;
			}
            *(uint16_t*)(out + 4) = outlen;
			MDMA_Channel_TypeDef* mc = ch->TxMdma;
			Mdma::InitHard(mc, 0x00000040, (ind < nguids) ? mlGuids[ind] : mlReply[0]);
			Mdma::Start(mc);
		    CleanDCacheIfUsed((uint32_t*)out, 12);
            break;
		}
		case MRC_UPGRADE: {
			// if (clen < 8) BREAK(MRE_FORMAT)
            // if (FlashIsLocked()) BREAK(MRE_FLASHLOCKED)
			// uint32_t op = *(uint16_t*)(buf+6), ind = *(uint16_t*)(buf+8), count = *(uint16_t*)(buf+10);
			// if ((count<<2) > PROGRAM_BLOCK) BREAK(MRE_FORMAT)
            // switch (op) {
            //     case 1:    // Erase
        	// 		if (clen != 12) BREAK(MRE_FORMAT)
            //         if (ind+count > 3) BREAK(MRE_ILLEGALINDEX)
            //         for (int i=0; i<count; i++) if (FlashUpgradeErase(ind+i) != 0) BREAK(MRE_FLASHERASEFAILED)
            //         break;
            //     case 2:     // Write
        	// 		if (clen != 12+(count<<2)) BREAK(MRE_FORMAT)
            //         if (FlashUpgradeWrite(ind<<6, (uint32_t*)(buf+12), count<<2) != 0) BREAK(MRE_FLASHWRITEFAILED)
            //         break;
            //     case 3:     // Validate
        	// 		if (clen != 20) BREAK(MRE_FORMAT)
            //         if ((ind != 0) || (count != 2)) BREAK(MRE_ILLEGALINDEX)
            //         if (FlashUpgradeInfo((uint32_t*)(buf+12)) != 0) BREAK(MRE_FLASHWRITEFAILED)
            //         if (FlashValidateUpgrade()) BREAK(MRE_FLASHCHECKSUM)
            //         break;
            //     case 4:     // Invalidate firmware and restart
        	// 		if ((clen != 12) || (ind != 0) || (count != 0)) BREAK(MRE_FORMAT)
            //         if (FlashDiscardFirmware() != 0) BREAK(MRE_FLASHWRITEFAILED)
            //         SysRestart();
            //         break;
            //     default:
            //         BREAK(MRE_ILLEGALCOMMAND)
            // }
			break;
		}
		case MRC_FUNCTION: {
			// if (clen < 12) BREAK(MRE_FORMAT)
			// uint16_t var = *(uint16_t*)&buf[6], ind = *(uint16_t*)&buf[8], func = *(uint16_t*)&buf[10];
			// out[1] = StartThreadFunction(var, ind, func, (int32_t*)&buf[12], (clen - 12) >> 2);
			break;
		}
		default: {
			BREAK(MRE_ILLEGALCOMMAND);
		}
	}
//	uint16_t len = rep->Sect[0].Bytes;
//	for (int i = 1; i < sect; i++) len += rep->Sect[i].Bytes;
//	*(uint16_t*)&out[4] = len;
//	rep->nSect = sect; 
	//rep->cSect = -1;
	ch->StartWrite(outlen);
}

void CommandExecute(uint8_t* com, int comcount, CommChannel* ch)
{
	if (com[0] == 0xFD) {
		FdProtocol(com, comcount, ch);
	}
}

void CommandInit() {
	const uint8_t* strs[nstrings] = {
		ProductString,
		ManufacturerString,
		0, // SerialNumberString, 
		GitVersion, // FwVersionString,
		0,
		0, // ApplicationString,
		CdfString, // CDF,
	};
	uint16_t tstrl = 0;
	const GUID* guids[nguids] = {
		&GuidProduct,
		&GuidManufacturer,
		&GuidUnit,
		&GuidFwVersion,
		0
	};
	// Copy strings up to CDF into SRAM2 buffer
	int i;
	for (i = 0; i < nstrings; i++) {
		DStringsLen[i] = ((strs[i] == 0) || (strs[i][0] == 0) || (strs[i][0] == 0xFF)) ? 1 : strlen((char*)strs[i]) + 1;
		if (i < 6) tstrl += DStringsLen[i];  // up to CDF
	}
	tstrl += 60; 	// add space for Application string
	DStrings[0] = (uint8_t*)AllocRamD2(tstrl);
	for (i = 0; i < 6-1; i++) DStrings[i+1] = DStrings[i] + DStringsLen[i];
	DStrings[6] = CdfString;
	for (i = 0; i < 6; i++) {
		if (DStringsLen[i] == 1) *DStrings[i] = 0; else memcpy(DStrings[i], strs[i], DStringsLen[i]);
	}
	// Build MDMA links for GUIDS
    uint32_t CTCR = 0x7000000A;        // Software request, TRGM Full Transfer, byte sizes, no burst, source/target increment 
	uint32_t *d = (uint32_t*)&GuidUnit, *s = (uint32_t*)0x1FF1E800;
	*d++ = 0x37484D20;	// MH7
	*d++ = *s++; *d++ = *s++; *d++ = *s++; 
	for (int i = 0; i < nguids; i++) {
		Mdma::InitLink(mlGuids[i], CTCR, 16, (uint32_t)((guids[i] == 0) ? &emptyguid : guids[i]));
	} 
 }
