// Command.cpp
// Communication protocol
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"
#include <string.h>

#include "Global.h"
//#include "SysVars.h"
//#include "Martel.h"
//#include "Cdf.h"
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

#define FdReplyMax  128
uint8_t FdReply[FdReplyMax] = {0xFD};

#define nstrings 10
__attribute__((section(".mdmalink"))) static struct MdmaLink mlStrings[nstrings];
__attribute__((section(".ramD1init"))) static uint8_t emptystring[] = "";
__attribute__((section(".mdmalink"))) static struct MdmaLink mlReply[4];

#define BREAK(e) {FdReply[1] = e; break;}


void FdProtocol(uint8_t* buf, uint16_t count, SendStruct* rep) {
	uint16_t id = *(uint16_t*)&buf[2], clen = *(uint16_t*)&buf[4];
	// uint8_t error = 0; 	// Positive acknowledge assumed
    rep->Flag = 0;
    rep->nSect = 1;
	rep->Sect[0].Buf = FdReply;
	FdReply[1] = 0;						// Success assumed
	*(uint16_t*)(FdReply + 2) = id;
	*(uint16_t*)(FdReply + 4) = rep->Sect[0].Bytes = 8;		// Default prompt assumed
	*(uint16_t*)(FdReply + 6) = 0;	// Default prompt assumed
	switch (buf[1]) {
		case MRC_READVAR: {
			// if (clen != 8) BREAK(MRE_FORMAT)
			// uint16_t var = *(uint16_t*)(buf + 6);
            // *(uint16_t*)(FdReply + 6) = var;
            // if (var < 10000) {      // System variable
            //     if ((var >= nSysVars) || (SysVars[var].Size <= 0)) BREAK(MRE_ILLEGALVARIABLE)
            //     *(uint16_t*)(FdReply + 6) = var;
            //     int n = SysVars[var].Size;
            //     if (n > 16) n = 16;
            //     *(uint16_t*)(FdReply + 4) = rep->Sect[0].Bytes = 8 + (n << 2);
            // } else if (var < 20000) { // User scalar
            //     var -= 10000;
            //     *(uint16_t*)(FdReply + 4) = rep->Sect[0].Bytes = 12;
            // } else if (var < 30000) { // User array
            //     var -= 20000;
            //     int32_t* ar = 0; // (int32_t*)MrtInfo.Arrays[var];
            //     int size = *ar++;
            //     if (size <= 16) {
            //         for (uint16_t i = 0; i < size; i++) *(uint32_t*)(FdReply + 8 + (i << 2)) = *ar++;
            //         *(uint16_t*)(FdReply + 4) = rep->Sect[0].Bytes = 8 + (size << 2);
            //     } else {
            //         rep->nSect = 2;
            //         *(uint16_t*)(FdReply + 4) = 8 + (size << 2);
            //         rep->Sect[0].Bytes = 8;
            //         rep->Sect[1].Buf = (uint8_t*)(ar + 1);
            //         rep->Sect[1].Bytes = size << 2;
            //     }
            // }	
			break;
		}
		case MRC_READARRAY: {
			// if (clen != 12) BREAK(MRE_FORMAT)
			// uint16_t var = *(uint16_t*)&buf[6], ind = *(uint16_t*)&buf[8], count = *(uint16_t*)&buf[10];
			// *(uint16_t*)(FdReply + 6) = var;
            // *(uint16_t*)(FdReply + 8) = ind;
            // *(uint16_t*)(FdReply + 10) = count;
            // if (var < 10000) {      // System variable
            //     if (var >= nSysVars) BREAK(MRE_ILLEGALVARIABLE)
			// 	Vardef* vd = SysVars + var;
            //     int n = vd->Size;
			// 	if (n == 0) BREAK(MRE_ILLEGALVARIABLE)
            //     if (ind + count > n) BREAK(MRE_ILLEGALINDEX)
			// 	if (vd->Flags & VF_DIRECTREAD) {
			// 		*(uint16_t*)(FdReply + 4) = 12 + (count << 2);
			// 		if (count <= 16) {
			// 		  	rep->Sect[0].Bytes = *(uint16_t*)(FdReply + 4);
            //             if ((vd->Flags & VF_TYPE) >= TYPE_INT32) {
			// 			    MemCpy32((uint32_t*)(FdReply + 12), vd->Array + ind, count); 
            //             } else {
            //                 for (int i=0; i<count; i++) *(int32_t*)(FdReply+12+4*i) = GetSysVar(var, ind+i);
            //             }
			// 		} else {
			// 		    rep->nSect = 2;
			// 			rep->Sect[0].Bytes = 12;
			// 			rep->Sect[1].Buf = (uint8_t*)(vd->Array + ind);
			// 			rep->Sect[1].Bytes = count << 2;
			// 		}
			// 	} else if (vd->Flags & VF_PROPREAD) {
            //         int n = SysVars[var].ReadWrite(ind, count, (uint32_t*)(FdReply + 12));
            //         if (n > 0) {    // n is number of 32-bit words
	        //             *(uint16_t*)(FdReply + 4) = rep->Sect[0].Bytes = 12 + (n << 2);
            //         } else {        // -n is number of sections
            //             rep->nSect = -n + 1;
            //             uint16_t total;
			// 			total = rep->Sect[0].Bytes = 12;
            //             rep->Flag = (uint16_t*)*(uint32_t*)(FdReply + 12);
            //             uint32_t *from = (uint32_t*)(FdReply + 16), *to = (uint32_t*)&rep->Sect[1];
            //             for (int i = 0; i < -n; i++) {
            //                 *to++ = *from++;
            //                 total += (uint16_t)*from;
            //                 *to++ = *from++;
            //             }
            //             *(uint16_t*)(FdReply + 4) = total;
            //         }
			// 	} else BREAK(MRE_ILLEGALVARIABLE)
			// } else if (var < 20000) {   // User scalar
            //     if (ind + count > 1) BREAK(MRE_ILLEGALINDEX)
            //     *(int32_t*)&FdReply[12] = MrtInfo.Globals[var-10000];
            //     *(uint16_t*)(FdReply + 4) = 16;
			// } else if (var < 30000) {   // User array
            //     int32_t *a = (int32_t*)MrtInfo.Arrays[var-20000], n = *a;
            //     if (ind + count > n) BREAK(MRE_ILLEGALINDEX)
            //     rep->nSect = 2;
            //     rep->Sect[0].Bytes = 12;
            //     rep->Sect[1].Buf = (uint8_t*)(a + ind);
            //     rep->Sect[1].Bytes = count << 2;
            // }
			break;
		}
		case MRC_WRITEARRAY: {
			// if (clen < 12) BREAK(MRE_FORMAT)
			// uint16_t var = *(uint16_t*)&buf[6], ind = *(uint16_t*)&buf[8], count = *(uint16_t*)&buf[10];
			// if (clen != 12 + count * 4) BREAK(MRE_FORMAT)
            // if (var < 10000) {      // System variable
            //     if (var >= nSysVars) BREAK(MRE_ILLEGALVARIABLE)
			// 	Vardef* vd = SysVars + var;
            //     int n = vd->Size;
			// 	if (n == 0) BREAK(MRE_ILLEGALVARIABLE)
            //     if (ind + count > n) BREAK(MRE_ILLEGALINDEX)
			// 	if (vd->Flags & VF_DIRECTWRITE) {
            //         if ((vd->Flags & VF_TYPE) >= TYPE_INT32) {
    		// 			MemCpy32(vd->Array + ind, (uint32_t*)(buf + 12), count); 
            //         } else {
            //             for (int i=0; i<count; i++) SetSysVar(var, ind+i, *(int32_t*)(buf+12+4*i));
            //         }
			// 	} else if (vd->Flags & VF_PROPWRITE) {
	        //         if (SysVars[var].ReadWrite(ind, -count, (uint32_t*)(buf + 12)) <= 0) BREAK(MRE_ILLEGALCOMMAND)
			// 	} else BREAK(MRE_READONLYVARIABLE)
			// } else if (var < 20000) {   // User array
            //     int32_t v = var - 10000;
            //     if (v >= MrtInfo.nGlobals) BREAK(MRE_ILLEGALVARIABLE)
            //     if (ind + count > 1) BREAK(MRE_ILLEGALINDEX)
            //     MrtInfo.Globals[v] = *(int32_t*)(buf+12);
			// } else if (var < 30000) {   // User array
            //     int32_t v = var - 20000;
            //     if (v >= MrtInfo.nArrays) BREAK(MRE_ILLEGALVARIABLE)
            //     int32_t *a = (int32_t*)MrtInfo.Arrays[v], n = *a;
            //     if (ind + count > n) BREAK(MRE_ILLEGALINDEX)
			// 	MemCpy32(a + ind + 1, (int32_t*)(buf + 12), count); 
			// }
			break;
		}
		case MRC_READMULTI: {
			// if (clen < 12) BREAK(MRE_FORMAT)
			// uint16_t* ip = (uint16_t*)&buf[6];
			// uint16_t count = *ip++;
			// if (count > 64) BREAK(MRE_FORMAT)
			// if (clen != 8 + 4 * count) BREAK(MRE_FORMAT)
			// int32_t* op = (int32_t*)&FdReply[8];
			// for (int i = 0; i < count; i++) {
			// 	uint16_t var = *ip++, ind = *ip++;
            //     if (var < 10000) {      // System variable
	    	// 		*op++ = GetSysVar(var, ind);
    		// 	} else if (var < 20000) {   // User scalar
            //         uint16_t v = var - 10000;
            //         *op++ = ((v < MrtInfo.nGlobals) && (ind == 0)) ? MrtInfo.Globals[v] : -1;                
    		// 	} else if (var < 30000) {   // User array
            //         uint16_t v = var - 20000;
            //         int32_t* a = (v < MrtInfo.nArrays) ? (int32_t*)MrtInfo.Arrays[v] : 0;
            //         *op++ = ((a > 0) && (ind < *a)) ? a[ind+1] : -1;                
            //     } else {
            //         *op++ = -1;                
            //     }
			// }	
			// *(uint16_t*)&FdReply[6] = count;
			// *(uint16_t*)&FdReply[4] = rep->Sect[0].Bytes = clen;
			break;
		}
		case MRC_READSTRING: {
			if (clen != 12) BREAK(MRE_FORMAT)
			uint16_t ind = *(uint16_t*)&buf[8], count = *(uint16_t*)&buf[10];
            if (count > 10) count = 10;
			uint8_t* d = CommChannels[0]->Outbuf;
			*d = 0xFD; *(d+1) = 0;
			*(uint16_t*)(d + 2) = id;
			*(uint16_t*)(d + 6) = 0;
            *(uint16_t*)(d + 8) = ind;
            *(uint16_t*)(d + 10) = count;
            int total = 12;
			struct MdmaLink* ml = (ind < nstrings) ? mlStrings + ind : 0;
			int i = 0, ii = ind;
            for ( ; (i < count) && (ii < nstrings); i++, ii++) {
				ml->CDAR = (uint32_t)(d + total);
				ml->CLAR = (uint32_t)(ml + 1);
				total += ml->CBNDTR & 0x0001FFFF;
				ml++;
				// if (i != 0) (ml-1)->CLAR = (uint32_t)ml;
				// int ii = ind + i, len = (ii < nstrings) ? strlens[ii] : 0;
				// ml->CTCR = 0x7000000A | (len << 18);	// Software request, TRGM Full Transfer, byte sizes, no burst, source/target increment 
				// ml->CBNDTR = 0x00100000 | (len + 1);				
				// ml->CSAR = (uint32_t)((len == 0) ? emptystring : strs[ii]);
				// ml->CDAR = (uint32_t)(d + total);
				// ml->CBRUR = 0;
				// ml->CLAR = 0;
				// ml->CTBR = (ml->CSAR < 0x20020000) ? 0x00010000 : 0x00000000;	// Source AXI/TCM bus, target AXI bus
				// ml->CMAR = 0;
				// ml->CMDR = 0;
				// total += len + 1;
			} 
			int fill = count - i + (4 - (total & 3) & 3);  // overindexing and alignment
			if (fill) {
				if (ml != 0) (ml-1)->CLAR = (uint32_t)(mlReply);
				Mdma::InitLink(mlReply[0], 0x70000008, fill, (uint32_t)emptystring, (uint32_t)(d + total));
				total += fill; 
			} else {
				(ml-1)->CLAR = 0;
			}
			MDMA_Channel_TypeDef* mc = ((CommChannelDma*)CommChannels[0])->TxMdma;
			Mdma::InitHard(mc, 0x00000040, (ind < nstrings) ? mlStrings[ind] : mlReply[0]);
			mc->CCR |= 0x00000001;	// Enable
			mc->CCR |= 0x00010000;	// Start


			// MDMA_Channel0->CCR = 0x00000040;	// Priority medium, interrupts disabled
			// mb = MdmaLinkBuf;
			// MDMA_Channel0->CTCR = mb->CTCR;
			// MDMA_Channel0->CBNDTR = mb->CBNDTR;
			// MDMA_Channel0->CSAR = mb->CSAR;
			// MDMA_Channel0->CDAR = mb->CDAR;
			// MDMA_Channel0->CBRUR = mb->CBRUR; 
			// MDMA_Channel0->CLAR = mb->CLAR;
			// MDMA_Channel0->CTBR = mb->CTBR;
			// MDMA_Channel0->CMAR = mb->CMAR;
			// MDMA_Channel0->CMDR = mb->CMDR;
			// MDMA_Channel0->CCR |= 0x00000001;	// Enable
			// MDMA_Channel0->CCR |= 0x00010000;	// Start
            *(uint16_t*)(d + 4) = total;
            break;
        }
		case MRC_WRITESTRING: {
			// if (clen < 12) BREAK(MRE_FORMAT)
			// uint16_t ind = *(uint16_t*)&buf[8], count = *(uint16_t*)&buf[10];
			// if ((ind != 2) || (count != 1)) BREAK(MRE_ILLEGALINDEX)
            // if (FlashIsLocked()) BREAK(MRE_FLASHLOCKED)
			// *(uint16_t*)&FdReply[6] = 0; *(uint16_t*)&FdReply[8] = 2; *(uint16_t*)&FdReply[10] = 1;
            // uint16_t r = FlashSaveSerial(&buf[12]);
            // if (r != 0) BREAK(MRE_FLASHWRITEFAILED)
			// rep->Sect[0].Bytes = 12;
			break;
		}
		case MRC_READGUID: {
			if (clen != 12) BREAK(MRE_FORMAT)
			uint16_t ind = *(uint16_t*)&buf[8], count = *(uint16_t*)&buf[10];
			if (ind + count > 5) BREAK(MRE_ILLEGALINDEX)
			*(uint16_t*)&FdReply[6] = 0; *(uint16_t*)&FdReply[8] = ind; *(uint16_t*)&FdReply[10] = ind;
			uint32_t* r = (uint32_t*)&FdReply[12];
			for (int i = 0; i < count; i++) {
				switch (ind + i) {
					case 0: MemCpy32(r, (uint32_t*)&GuidProduct, 4); break;
					case 1: MemCpy32(r, (uint32_t*)&GuidManufacturer, 4); break;
					case 2: GetGuidUnit(r); break;
					case 3: GuidFromString((GUID*)r, GitSha); break;
					// case 4: MemCpy32(r, MrtGuid(), 4); ; break;
				}
				r += 4;
			}
			*(uint16_t*)(FdReply + 4) = rep->Sect[0].Bytes = 12 + (count << 4);
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
			// FdReply[1] = StartThreadFunction(var, ind, func, (int32_t*)&buf[12], (clen - 12) >> 2);
			break;
		}
		default: {
			FdReply[1] = MRE_ILLEGALCOMMAND;
		}
	}
//	uint16_t len = rep->Sect[0].Bytes;
//	for (int i = 1; i < sect; i++) len += rep->Sect[i].Bytes;
//	*(uint16_t*)&FdReply[4] = len;
//	rep->nSect = sect; 
	//rep->cSect = -1;
	CommChannels[0]->StartWrite(CommChannels[0]->Outbuf, *(uint16_t*)(CommChannels[0]->Outbuf + 4));
}

void ExecuteCommand(uint8_t ConAddr, uint8_t* ComBuf, int received, SendStruct* rep)
{
	if (ComBuf[0] == 0xFD) {
		FdProtocol(ComBuf, received, rep);
	} else if (ComBuf[0] == 0xE4) {
		
	}
}

void CommandInit() {
	const uint8_t* strs[nstrings] = {
		ProductString,
		ManufacturerString,
		0, // FlashGetSerial(),
		GitVersion,
		ApplicationString,
		0, // MotorString,
		0, // CDF,
	};
	uint32_t CTCR = 0x7000000A;		// Software request, TRGM Full Transfer, byte sizes, no burst, source/target increment 
	for (int i = 0; i < nstrings; i++) {
		int len = (strs[i] == 0) ? 0 : strlen((char*)strs[i]);
		Mdma::InitLink(mlStrings[i], CTCR | (len << 18), len + 1, (uint32_t)((len == 0) ? emptystring : strs[i]));
	} 
 }
