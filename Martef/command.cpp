// Command.cpp
// Communication protocol
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"
#include <string.h>

#include "global.h"
#include "sysvar.h"
#include "flash.h"
#include "gitversion.h"
#include "communication.h"
#include "command.h"

//  FD protocol:
//	Byte 0: FD
//	Byte 1: Command/reply
//	Byte 2,3: ID
//	Byte 4,5: Length

//  E3 protocol
//	Byte 0: E3
//	Byte 1: Number of sections
//	Byte 2: Section number
//	Byte 3,4,5: Total length
//	Byte 6,7: Section length
//	Byte 8,9: Section offset

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

#define nstrings 7
// uint8_t* DStrings[nstrings];
// uint16_t DStringsLen[nstrings];
// MultiBufStruct Strings;
// MultiBufStruct Guids;

/*__attribute__((section(".ramD1init")))*/ const uint8_t ProductString[] = "Martef-H7";
/*__attribute__((section(".ramD1init")))*/ const uint8_t ManufacturerString[] = "XACT Robotics";
__attribute__((section(".ramD1"))) uint8_t SerialNumberString[60];
__attribute__((section(".ramD1"))) uint8_t ApplicationString[60];
/*__attribute__((section(".ramD2init")))*/const uint8_t CdfString[] = 
	#include "xact.cdc"
	"\x00\x00\x00"	// for alignment
;
__attribute__((section(".ramD1init"))) static uint8_t emptystring[1] = { 0 };
#define nguids 5
const GUID GuidProduct = { 0x6729d920, 0xcb01, 0x4119, { 0x90, 0x5e, 0xc5, 0x55, 0x2b, 0xea, 0x14, 0x1c } }; // {6729D920-CB01-4119-905E-C5552BEA141C}
const GUID GuidManufacturer = { 0xebb27f3b, 0x758f, 0x4dbe, { 0x94, 0x93, 0x4, 0x9f, 0xec, 0x2e, 0x8f, 0x22 } };  // {EBB27F3B-758F-4DBE-9493-049FEC2E8F22}
__attribute__((section(".ramD1"))) GUID GuidUnit;
__attribute__((section(".ramD1init"))) GUID GuidFwVersion { 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0 } };
const static GUID emptyguid { 0, 0, 0, { 0, 0, 0, 0, 0, 0, 0, 0 } };

const uint8_t* Strings[nstrings] = {
	ProductString,
	ManufacturerString,
	0, //SerialNumberString, 
	GitVersion, 
	0,
	0, //ApplicationString,
	CdfString, // Controller Definition File
};
uint16_t tstrl = 0;
const GUID* Guids[nguids] = {
	&GuidProduct,
	&GuidManufacturer,
	&GuidUnit,
	&GuidFwVersion,
	0
};

#define BREAK(e) {rep[1] = e; break;}

void FdProtocol(TransactionStruct* t) {
	uint8_t* com = t->Inb;
	uint16_t id = *(uint16_t*)&com[2], clen = *(uint16_t*)&com[4];
	uint8_t* rep = t->Outb;			// Reply buffer
	uint16_t repl = 8;				// Reply length or partial reply length in case of SaDma
	rep[0] = 0xFD; rep[1] = 0;		// Success assumed
	*(uint16_t*)(rep + 2) = id;
	*(uint16_t*)(rep + 4) = 8;		// Default prompt assumed
	*(uint16_t*)(rep + 6) = 0;		// Default prompt assumed
	t->Mbuf.Count = 1;
	t->Mbuf.TerminationFlag = 0;
	t->Mbuf.Bufs[0].Addr = rep;		// Default prompt assumed
	t->Mbuf.Bufs[0].Count = 8;		// Default prompt assumed
	switch (com[1]) {
		case MRC_READVAR: {
			// if (clen != 8) BREAK(MRE_FORMAT)
			// uint16_t var = *(uint16_t*)(com + 6);
            // *(uint16_t*)(rep + 6) = var;
            // if (var < 10000) {      // System variable
            //     if ((var >= nSysVars) || (SysVars[var].Size <= 0)) BREAK(MRE_ILLEGALVARIABLE)
            //     *(uint16_t*)(rep + 6) = var;
            //     int n = SysVars[var].Size;
            //     if (n > 16) n = 16;
            //     *(uint16_t*)(rep + 4) = rep->Sect[0].Bytes = 8 + (n << 2);
            // } else if (var < 20000) { // User scalar
            //     var -= 10000;
            //     *(uint16_t*)(rep + 4) = rep->Sect[0].Bytes = 12;
            // } else if (var < 30000) { // User array
            //     var -= 20000;
            //     int32_t* ar = 0; // (int32_t*)MrtInfo.Arrays[var];
            //     int size = *ar++;
            //     if (size <= 16) {
            //         for (uint16_t i = 0; i < size; i++) *(uint32_t*)(rep + 8 + (i << 2)) = *ar++;
            //         *(uint16_t*)(rep + 4) = rep->Sect[0].Bytes = 8 + (size << 2);
            //     } else {
            //         rep->nSect = 2;
            //         *(uint16_t*)(rep + 4) = 8 + (size << 2);
            //         rep->Sect[0].Bytes = 8;
            //         rep->Sect[1].com = (uint8_t*)(ar + 1);
            //         rep->Sect[1].Bytes = size << 2;
            //     }
            // }	
			break;
		}
		case MRC_READARRAY: {
			if (clen != 12) BREAK(MRE_FORMAT)
			uint16_t var = *(uint16_t*)(com+6), ind = *(uint16_t*)(com+8), count = *(uint16_t*)(com+10);
			*(uint16_t*)(rep + 6) = var;
            *(uint16_t*)(rep + 8) = ind;
            if (var < 10000) {      // System variable
                if (var >= nSysVars) BREAK(MRE_ILLEGALVARIABLE)
				Vardef* vd = SysVars + var;
                int n = vd->size;
				if (n == 0) BREAK(MRE_ILLEGALVARIABLE)
                if (ind > n) BREAK(MRE_ILLEGALINDEX)
				int32_t acount = vd->read(ind, count, (int32_t*)(rep+12));
				if (acount >= 0) {
		            *(uint16_t*)(rep + 10) = acount;
					repl = 12 + (count << 2);
					*(uint16_t*)(rep+4) = repl;
					t->Mbuf.Bufs[0].Count = repl;
				} else {
					MultiBufStruct* mb = (MultiBufStruct*)*(uint32_t*)(rep+12);
					t->Mbuf.Bufs[0].Count = 12;
					t->Mbuf.Count = mb->Count + 1;
					t->Mbuf.TerminationFlag = mb->TerminationFlag;
					MemCpy32(&t->Mbuf.Bufs[1], mb->Bufs, mb->Count << 1);
					uint16_t len = 0;
					for (int i = 0; i < mb->Count; i++) len += mb->Bufs[i].Count;
		            *(uint16_t*)(rep + 10) = len >> 2;
		            *(uint16_t*)(rep + 4) = 12 + len;
				}
			} else if (var < 20000) {   // User scalar
                // if (ind + count > 1) BREAK(MRE_ILLEGALINDEX)
                // *(int32_t*)&rep[12] = MrtInfo.Globals[var-10000];
                // *(uint16_t*)(rep + 4) = 16;
			} else if (var < 30000) {   // User array
                // int32_t *a = (int32_t*)MrtInfo.Arrays[var-20000], n = *a;
                // if (ind + count > n) BREAK(MRE_ILLEGALINDEX)
                // rep->nSect = 2;
                // rep->Sect[0].Bytes = 12;
                // rep->Sect[1].com = (uint8_t*)(a + ind);
                // rep->Sect[1].Bytes = count << 2;
            }
		    CleanDCacheIfUsed((uint32_t*)rep, repl);
			break;
		}
		case MRC_WRITEARRAY: {
			if (clen < 12) BREAK(MRE_FORMAT)
			uint16_t var = *(uint16_t*)(com+6), ind = *(uint16_t*)(com+8), count = *(uint16_t*)(com+10);
			if (clen != 12 + count * 4) BREAK(MRE_FORMAT)
            if (var < 10000) {      // System variable
                if (var >= nSysVars) BREAK(MRE_ILLEGALVARIABLE)
				Vardef* vd = SysVars + var;
                int n = vd->size;
				if (n == 0) BREAK(MRE_ILLEGALVARIABLE)
                // if (ind + count > n) BREAK(MRE_ILLEGALINDEX)
				if (vd->write == 0) BREAK(MRE_READONLYVARIABLE)
				vd->write(ind, count, (int32_t*)(com+12));
			} else if (var < 20000) {   // User array
                // int32_t v = var - 10000;
                // if (v >= MrtInfo.nGlobals) BREAK(MRE_ILLEGALVARIABLE)
                // if (ind + count > 1) BREAK(MRE_ILLEGALINDEX)
                // MrtInfo.Globals[v] = *(int32_t*)(com+12);
			} else if (var < 30000) {   // User array
                // int32_t v = var - 20000;
                // if (v >= MrtInfo.nArrays) BREAK(MRE_ILLEGALVARIABLE)
                // int32_t *a = (int32_t*)MrtInfo.Arrays[v], n = *a;
                // if (ind + count > n) BREAK(MRE_ILLEGALINDEX)
				// MemCpy32(a + ind + 1, (int32_t*)(com + 12), count); 
			}
		    CleanDCacheIfUsed((uint32_t*)rep, 8);
			break;
		}
		case MRC_READMULTI: {
GPIOF->BSRR = 0x00002000;                  // F13 = 1 
			if (clen < 12) BREAK(MRE_FORMAT)
			uint16_t* ip = (uint16_t*)(com+6);
			uint16_t count = *ip++;
			if (count > 64) BREAK(MRE_FORMAT)
			if (clen != 8 + 4 * count) BREAK(MRE_FORMAT)
			int32_t* op = (int32_t*)(rep+8);
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
			*(uint16_t*)(rep+6) = count;
			repl = 8 + (count << 2);
			*(uint16_t*)(rep+4) = repl;
			t->Mbuf.Bufs[0].Count = repl;
			t->Mbuf.Count = 1;
			t->Mbuf.TerminationFlag = 0;
		    CleanDCacheIfUsed((uint32_t*)rep, repl);
GPIOF->BSRR = 0x20000000;                  // F13 = 0 
			break;
		}
		case MRC_READSTRING: {
			if (clen != 12) BREAK(MRE_FORMAT)
			uint16_t ind = *(uint16_t*)&com[8], count = *(uint16_t*)&com[10];
			if (ind >= nstrings) BREAK(MRE_WRONGINDEX)
			int16_t rem = nstrings - ind; 
			if (count > rem) count = rem;
            int repl = 0;
			uint8_t** s = (uint8_t**)Strings + ind;
			BufStruct* d = t->Mbuf.Bufs;
			d++->Count = 12;
			for (int i = 0; i < count; i++,s++,d++) { 
				uint8_t* s1 = ((*s == 0) || (**s == 0xFF)) ? emptystring : *s;
				int n = strlen((const char*)s1) + 1;
				repl += n;  
				d->Addr = s1;  
				d->Count = n;
			}
			t->Mbuf.Count = count + 1;
            *(uint16_t*)(rep + 4) = 12 + repl;
            *(uint16_t*)(rep + 6) = 0;
            *(uint16_t*)(rep + 8) = ind;
            *(uint16_t*)(rep + 10) = count;
		    CleanDCacheIfUsed((uint32_t*)rep, 12);
            break;
        }
		case MRC_WRITESTRING: {
			// if (clen < 12) BREAK(MRE_FORMAT)
			// uint16_t ind = *(uint16_t*)&com[8], count = *(uint16_t*)&com[10];
			// if ((ind != 2) || (count != 1)) BREAK(MRE_ILLEGALINDEX)
            // if (FlashIsLocked()) BREAK(MRE_FLASHLOCKED)
			// *(uint16_t*)&rep[6] = 0; *(uint16_t*)&rep[8] = 2; *(uint16_t*)&rep[10] = 1;
            // uint16_t r = FlashSaveSerial(&com[12]);
            // if (r != 0) BREAK(MRE_FLASHWRITEFAILED)
			// rep->Sect[0].Bytes = 12;
			break;
		}
		case MRC_READGUID: {
			if (clen != 12) BREAK(MRE_FORMAT)
			uint16_t ind = *(uint16_t*)&com[8], count = *(uint16_t*)&com[10];
			if (ind >= nguids) BREAK(MRE_WRONGINDEX)
			if (ind + count > nguids) count = nguids - ind;
            *(uint16_t*)(rep + 4) = 12 + (count << 4);
            *(uint16_t*)(rep + 6) = 0;
			*(uint16_t*)(rep + 8) = ind; 
			*(uint16_t*)(rep + 10) = count;
			GUID** s = (GUID**)Guids + ind; 
			BufStruct* d = t->Mbuf.Bufs;
			d++->Count = 12;
			for (int i = 0; i < count; i++,s++,d++) { 
				d->Addr = (uint8_t*)*s;  
				d->Count = 16;
			}
			t->Mbuf.Count = count + 1;
            break;
		}
		case MRC_UPGRADE: {
			if (clen < 8) BREAK(MRE_FORMAT)
            if (FlashIsLocked()) BREAK(MRE_FLASHLOCKED)
			uint32_t op = *(uint16_t*)(com+6), ind = *(uint16_t*)(com+8), count = *(uint16_t*)(com+10);
			if ((count<<2) > PROGRAM_BLOCK) BREAK(MRE_FORMAT)
            switch (op) {
                case 1:    // Erase
        			if (clen != 12) BREAK(MRE_FORMAT)
                    if (ind+count > 3) BREAK(MRE_ILLEGALINDEX)
                    for (uint32_t i=0; i<count; i++) if (FlashUpgradeErase(ind+i) != 0) BREAK(MRE_FLASHERASEFAILED)
                    break;
                case 2:     // Write
        			if (clen != 12+(count<<2)) BREAK(MRE_FORMAT)
                    if (FlashUpgradeWrite(ind<<6, (uint32_t*)(com+12), count<<2) != 0) BREAK(MRE_FLASHWRITEFAILED)
                    break;
                case 3:     // Validate
        			if (clen != 20) BREAK(MRE_FORMAT)
                    if ((ind != 0) || (count != 2)) BREAK(MRE_ILLEGALINDEX)
                    if (FlashUpgradeInfo((uint32_t*)(com+12)) != 0) BREAK(MRE_FLASHWRITEFAILED)
                    if (FlashValidateUpgrade()) BREAK(MRE_FLASHCHECKSUM)
                    break;
                case 4:     // Invalidate firmware and restart
        			if ((clen != 12) || (ind != 0) || (count != 0)) BREAK(MRE_FORMAT)
                    if (FlashDiscardFirmware() != 0) BREAK(MRE_FLASHWRITEFAILED)
                    SysRestart();
                    break;
                default:
                    BREAK(MRE_ILLEGALCOMMAND)
            }
			break;
		}
		case MRC_FUNCTION: {
			// if (clen < 12) BREAK(MRE_FORMAT)
			// uint16_t var = *(uint16_t*)&com[6], ind = *(uint16_t*)&com[8], func = *(uint16_t*)&com[10];
			// rep[1] = StartThreadFunction(var, ind, func, (int32_t*)&com[12], (clen - 12) >> 2);
			break;
		}
		default: {
			BREAK(MRE_ILLEGALCOMMAND);
		}
	}
}

void CommandExecute(TransactionStruct* t)
{
	if (t->Inb[0] == 0xE3) { t->Inb += 10; t->Inbl -= 10; }

	if (t->Inb[0] == 0xFD) {
// GPIOF->BSRR = 0x00002000;                  // F13 = 1
  		FdProtocol(t);
// GPIOF->BSRR = 0x20000000;                  // F13 = 0
  	}
}

void CommandInit() {
	// const uint8_t* strs[nstrings] = {
	// 	ProductString,
	// 	ManufacturerString,
	// 	0, // SerialNumberString, 
	// 	GitVersion, // FwVersionString,
	// 	0,
	// 	0, // ApplicationString,
	// 	CdfString, // CDF,
	// };
	// uint16_t tstrl = 0;
	// const GUID* guids[nguids] = {
	// 	&GuidProduct,
	// 	&GuidManufacturer,
	// 	&GuidUnit,
	// 	&GuidFwVersion,
	// 	0
	// };
	// // Copy strings up to CDF into SRAM2 buffer
	// int i;
	// for (i = 0; i < nstrings; i++) {
	// 	DStringsLen[i] = ((strs[i] == 0) || (strs[i][0] == 0) || (strs[i][0] == 0xFF)) ? 1 : strlen((char*)strs[i]) + 1;
	// 	if (i < 6) tstrl += DStringsLen[i];  // up to CDF
	// }
	// tstrl += 60; 	// add space for Application string
	// DStrings[0] = (uint8_t*)AllocRamD2(tstrl);
	// for (i = 0; i < 6-1; i++) DStrings[i+1] = DStrings[i] + DStringsLen[i];
	// DStrings[6] = (uint8_t*)CdfString;
	// for (i = 0; i < 6; i++) {
	// 	if (DStringsLen[i] == 1) *DStrings[i] = 0; else memcpy(DStrings[i], strs[i], DStringsLen[i]);
	// }
	// // Build MDMA links for GUIDS
    // uint32_t CTCR = 0x7000000A;        // Software request, TRGM Full Transfer, byte sizes, no burst, source/target increment 
	uint32_t *d = (uint32_t*)&GuidUnit, *s = (uint32_t*)0x1FF1E800;
	*d++ = 0x37484D20;	// MH7
	*d++ = *s++; *d++ = *s++; *d++ = *s++; 
	// for (int i = 0; i < nguids; i++) {
	// 	Mdma::InitLink(mlGuids[i], CTCR, 16, (uint32_t)((guids[i] == 0) ? &emptyguid : guids[i]));
	// } 
 }
