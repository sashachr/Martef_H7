// Copyright (c) Sasha (Alexander) Chrichoff. All rights reserved.
// Under no circumstances may you delete or modify this file header.
// See LICENSE.TXT file in the project root for full license information.  
//
#include "chip.h"
#include <string.h>

#include "global.h"
#include "motion.h"
#include "servo.h"
#include "io.h"
#include "timer.h"
#include "adc.h"
//#include "Dac.h"
//#include "Flash.h"
//#include "Functions.h"
#include "systick.h"
#include "communication.h"
#include "triggerscope.h"
#include "siggen.h"
#include "flash.h"
//#include "Martel.h"
#include "martef.h"
#include "sysvar.h"

// This RW stuff is needed as our IAR version doesn't support lambdas
// static int32_t RwUint8(uint16_t ind, int16_t count, uint32_t* buf, uint8_t* var) {
// 	uint8_t* v = var + ind;
//   	if (count == 1) { 	// Read
// 	  	*buf = (uint32_t)*v;
//   	} else if (count == -1) { 	// Write
// 	  	count = 1;
// 	  	*v = (uint8_t)*buf;
//   	} else if (count > 0) { 	// Read
// 	  	for (int i=0; i<count; i++) *buf++ = (uint32_t)*v++;
// 	} else {    // Write
// 	  	count = -count;
// 	  	for (int i=0; i<count; i++) *v++ = (uint8_t)*buf++; 
// 	}
// 	return count;
// }
static int32_t RwBit(uint16_t ind, uint16_t count, uint32_t* buf, uint32_t* var, uint32_t bit) {
	uint32_t* v = var + ind;
  	if (count == 1) { 	// Read
	  	*buf = (*v & bit) != 0;
  	} else if (count == -1) { 	// Write
	  	count = 1;
	  	if (*buf) *v |= bit; else *v &= ~bit;
  	} else if (count > 0) { 	// Read
	  	for (int i=0; i<count; i++) *buf++ = (*v++ & bit) != 0;
	} else {    // Write
	  	count = -count;
	  	for (int i=0; i<count; i++) if (*buf++) *v++ |= bit; else *v++ &= ~bit;
	}
	return count;
}
//static int32_t RwLinearMode(uint16_t ind, int16_t count, uint32_t* buf) {return RwBit(ind, count, buf, &Servo.Mode, SM_LINEAR);}
//static int32_t RwUhrMode(uint16_t ind, int16_t count, uint32_t* buf) {return RwBit(ind, count, buf, &Servo.Mode, SM_UHR);}
//static int32_t RwDcMode(uint16_t ind, int16_t count, uint32_t* buf) {return RwBit(ind, count, buf, &Servo.Mode, SM_DC);}
//static int32_t RwSetOffset(uint16_t ind, int16_t count, uint32_t* buf) {return RwBit(ind, count, buf, &Servo.SetOffset);}
// static int32_t RwInTransition(uint16_t ind, int16_t count, uint32_t* buf) {return RwBit(ind, count, buf, &Servo.Mode);}
//static int32_t RwEnable(uint16_t ind, int16_t count, uint32_t* buf) {return RwBit(ind, count, buf, &Servo.Mode, SM_ENABLE);}

#define Direct(Var, Type, Size)     (Size), (Type)|VF_DIRECTREAD|VF_DIRECTWRITE, &Var, 0 
#define RwUint8(Var, Size)      Size, TYPE_UINT8|VF_PROPREAD|VF_PROPWRITE, 0, \
    [](uint16_t ind, int16_t count, uint32_t* buf) -> { \
        uint8_t* var = Var + ind; \
        for (int i=0; i<count; i++) *buf++ = Int32FromInt8(*var++); \
    }

int CopyString(uint8_t* dest, const uint8_t* source, int max) {
    int n = strlen((char*)source) + 1;
    if (n > max) n = max;
    strncpy((char*)dest, (char*)source, n);
    return n;
}
// int GetUnitString(uint8_t* buf, int count) {
//     return GetGuidString((GUID*)FlashGetUnitGuid(), buf, count);      
// }
int32_t GetStatus(uint16_t ind, uint16_t count, int32_t* buf) {
    for (int i=0; i<count; i++) {
        switch (ind+i) {
            case 0: {   // General status
                uint32_t stat = (Scope.Stage << 28);
                if (Scope.SendBuf >= 0) stat |= 0x80000000;
                *buf++ = stat;
				break;
            }
            case 1: {   // Program status
                *buf++ = 0;
				break;
            }
            case 3: {   // Driver status
                uint32_t s = 0;
//                s = Servo.Enabled() | (Servo.LinearMode()<<1) | (Servo.UhrMode()<<2) | (Servo.DcMode()<<3) | (Servo.InTransition<<5) | ((Servo.Safety != 0) << 8) | ((Servo.Error != 0) << 9);
                *buf++ = s;
				break;
            }
            default: *buf++ = 0;
        }
    }
    return count;
}
int32_t GetAxisStatus(uint16_t ind, uint16_t count, int32_t* buf) {
    for (int i=0; i<count; i++) {
        int a = ind + i;
        if (a < NAX) {
            *buf++ = 0;
        } else {
            *buf++ = 0;
        }
    }
    return count;
}
int32_t FlashKey(uint16_t ind, int16_t count, uint32_t* buf) {
    if (count == 1) {   // read
        // *buf = FlashIsLocked() ? 0 : 0x27A284F9;
    } else if (count == -1) {    // write
        // if (*buf == 0x27A284F9) FlashUnlock(); else FlashLock();
    }
    return 1;
}
int32_t FlashSave(uint16_t ind, int16_t count, uint32_t* buf) {
    if (count == 1) {       // Read
        // *buf = FlashOperationInProgress;
    } else if (count == -1) {   // save/erase
        if (*buf == 1) {    // save
            // if (FlashSaveInitArea() != 0) return 0;
        } else if (*buf == 2) {    // erase
            // if (FlashEraseInitArea() != 0) return 0;
        }
    }

    return 1;
}

#define StructScalarDirectRead(struc, var) \
    [](uint16_t i)->int32_t* {return (int32_t*)&struc.var;}, \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < 1); i++,j++) *buf++ = *(int32_t*)&struc.var; \
        for (; i < count; i++) *buf++ = NONE; \
        return i; \
    }
#define StructScalarDirectReadWrite(struc, var) \
    StructScalarDirectRead(struc, var), \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t { *(int32_t*)&struc.var = *buf++; }

#define StructScalarDirectReadArray(struc, var, maxind) \
    [](uint16_t i)->int32_t* {return (int32_t*)&struc.var[i];}, \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < maxind); i++,j++) *buf++ = *(int32_t*)&struc.var[j]; \
        for (; i < count; i++) *buf++ = NONE; \
        return i; \
    }
#define StructScalarDirectReadWriteArray(struc, var, maxind) \
    StructScalarDirectReadArray(struc, var), \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < maxind); i++,j++) if (!IsNan(*buf)) *(int32_t*)&struc.var[j] = *buf++; \
        return i; \
    }

#define StructDirectRead(struc, var, maxind) \
    [](uint16_t i)->int32_t* {return (int32_t*)&struc[i].var;}, \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < maxind); i++,j++) *buf++ = *(int32_t*)&struc[j].var; \
        for (; i < count; i++) *buf++ = NONE; \
        return i; \
    }
#define StructDirectReadWrite(struc, var, maxind) \
    StructDirectRead(struc, var, maxind), \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < maxind); i++,j++) if (!IsNan(*buf)) *(int32_t*)&struc[j].var = *buf++; \
        return i; \
    }
#define StructPropWrite(struc, func, maxind, argtype) \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < maxind); i++,j++) if (!IsNan(*buf)) struc[j].func(*(argtype*)buf++); \
        return i; \
    }

#define ServoDirectRead(var) StructDirectRead(Servo, var, NAX)
#define ServoDirectReadWrite(var) StructDirectReadWrite(Servo, var, NAX)
#define ServoPropWrite(func, argtype) StructPropWrite(Servo, func, NAX, argtype)
#define MotionDirectRead(var) StructDirectRead(Motions, var, NAX)
#define MotionDirectReadWrite(var) StructDirectReadWrite(Motions, var, NAX)
#define MotionPropWrite(func, argtype) StructPropWrite(Motions, func, NAX, argtype)

#define StructDirectReadShort(struc, var, maxind) \
    [](uint16_t i)->int32_t* {return (int32_t*)&struc[i].var;}, \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < maxind); i++,j++) *buf++ = struc[j].var; \
        for (; i < count; i++) *buf++ = NONE; \
        return i; \
    }
#define StructDirectReadWriteShort(struc, var, maxind, type) \
    StructDirectRead(struc, var, maxind), \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < maxind); i++,j++) if (!IsNan(*buf)) struc[j].var = *(type*)buf++; \
        return i; \
    }
#define StructReadBit(struc, var, maxind, mask) \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < maxind); i++,j++) *buf++ = (struc[j].var & mask) != 0; \
        for (; i < count; i++) *buf++ = NONE; \
        return i; \
    }
#define StructWriteBit(struc, var, maxind, mask) \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < maxind); i++,j++) if (*buf == 0) struc[j].var &= ~mask; else if (*buf == 1) struc[j].var |= mask; \
        return i; \
    }
#define WriteSignalRout(var) \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < NAX); i++,j++,buf++) Servo[j].SetSignalRout(var, (uint8_t)*buf); \
        return i; \
    }
#define WriteFpos \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < NAX); i++,j++,buf++) Servo[j].SetFpos(*(float*)buf); \
        return i; \
    }
#define WriteBqEnable(bqn) \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < NAX); i++,j++,buf++) { Servo[j].Vloop.Bq[bqn].Enable = *buf != 0; Servo[j].Vloop.Bq[bqn].SetShadow(); } \
        return i; \
    }
#define WriteBqMode(bqn) \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < NAX); i++,j++,buf++) Servo[j].Vloop.Bq[bqn].Config(*(int*)buf);  \
        return i; \
    }
#define WriteBqRaw(bqn, par) \
    [](uint16_t ind, uint16_t count, int32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < NAX); i++,j++,buf++) { Servo[j].Vloop.Bq[bqn].par = (*(float*)buf); if (Servo[j].Vloop.Bq[bqn].Enable) Servo[j].Vloop.Bq[bqn].s ## par = (*(float*)buf); Servo[j].Vloop.Bq[bqn].Mode = 3;} \
        return i; \
    }

Vardef SysVars[nSysVars] = {
    #include "sysvar.inc"
};

// int32_t GetSysVar(uint16_t id, uint16_t ind) {
// 	if ((id >= nSysVars) || (SysVars[id].Size == 0) || (ind >= SysVars[id].Size)) return -1;
// 	if (SysVars[id].Flags & VF_DIRECTREAD) {
//         switch (SysVars[id].Flags & VF_TYPE) {
//             case TYPE_INT8: return *((int8_t*)SysVars[id].Array + ind);
//             case TYPE_UINT8: return *((uint8_t*)SysVars[id].Array + ind);
//             case TYPE_INT16: return *((int16_t*)SysVars[id].Array + ind);
//             case TYPE_UINT16: return *((uint16_t*)SysVars[id].Array + ind);
//             default: return *(SysVars[id].Array + ind); 
//         }
// 	}
// 	if (SysVars[id].Flags & VF_PROPREAD) {
// 	  	uint32_t v;	
// 	  	SysVars[id].ReadWrite(ind, 1, &v);
// 		return v;
// 	}
// 	return -1;
// }

// int32_t SetSysVar(uint16_t id, uint16_t ind, uint32_t val) {
// 	if ((id >= nSysVars) || (SysVars[id].Size == 0) || (ind >= SysVars[id].Size)) return -1;
// 	if (SysVars[id].Flags & VF_DIRECTWRITE) {
//         switch (SysVars[id].Flags & VF_TYPE) {
//             case TYPE_INT8: *((int8_t*)SysVars[id].Array + ind) = *(int8_t*)&val; break;
//             case TYPE_UINT8: *((uint8_t*)SysVars[id].Array + ind) = *(uint8_t*)&val; break;
//             case TYPE_INT16: *((int16_t*)SysVars[id].Array + ind) = *(int16_t*)&val; break;
//             case TYPE_UINT16: *((uint16_t*)SysVars[id].Array + ind) = *(uint16_t*)&val; break;
//             default: *(SysVars[id].Array + ind) = val; 
//         }
//         return 0;
//     }
// 	if (SysVars[id].Flags & VF_PROPWRITE) {SysVars[id].ReadWrite(ind, -1, &val); return 0;}
// 	return -1;
// }

// int32_t GetSysArray(uint16_t id, uint16_t ind, uint16_t count, int32_t* buf) {
// 	if ((id >= nSysVars) || (SysVars[id].Size == 0) || (ind + count > SysVars[id].Size)) return -1;
// 	if (SysVars[id].Flags & VF_DIRECTREAD) {
//         if ((SysVars[id].Flags & VF_TYPE) >= TYPE_INT32) {
//             MemCpy32(buf + ind, SysVars[id].Array + ind, count);
//         } else {
//             switch (SysVars[id].Flags & VF_TYPE) {
//                 case TYPE_INT8: for (int i = 0; i < count; i++) buf[ind + i] = *((int8_t*)SysVars[id].Array + (ind + i));
//                 case TYPE_UINT8: for (int i = 0; i < count; i++) buf[ind + i] = *((uint8_t*)SysVars[id].Array + (ind + i));
//                 case TYPE_INT16: for (int i = 0; i < count; i++) buf[ind + i] = *((int16_t*)SysVars[id].Array + (ind + i));
//                 case TYPE_UINT16: for (int i = 0; i < count; i++) buf[ind + i] = *((uint16_t*)SysVars[id].Array + (ind + i));
//                 default: return -1; 
//             }
//         }
//         return count;
// 	}
// 	if (SysVars[id].Flags & VF_PROPREAD) {
//   	    return SysVars[id].ReadWrite(ind, count, (uint32_t*)(buf + ind));
// 	}
// 	return -1;
// }

// int32_t SetSysArray(uint16_t id, uint16_t ind, uint16_t count, int32_t* buf) {
// 	if ((id >= nSysVars) || (SysVars[id].Size == 0) || (ind + count > SysVars[id].Size)) return -1;
// 	if (SysVars[id].Flags & VF_DIRECTWRITE) {
//         if ((SysVars[id].Flags & VF_TYPE) >= TYPE_INT32) {
//             MemCpy32(SysVars[id].Array + ind, buf + ind, count);
//         } else {
//             switch (SysVars[id].Flags & VF_TYPE) {
//                 case TYPE_INT8: for (int i = 0; i < count; i++) *((int8_t*)SysVars[id].Array + (ind + i)) = buf[ind + i];
//                 case TYPE_UINT8: for (int i = 0; i < count; i++) *((uint8_t*)SysVars[id].Array + (ind + i)) = buf[ind + i];
//                 case TYPE_INT16: for (int i = 0; i < count; i++) *((int16_t*)SysVars[id].Array + (ind + i)) = buf[ind + i];
//                 case TYPE_UINT16: for (int i = 0; i < count; i++) *((uint16_t*)SysVars[id].Array + (ind + i)) = buf[ind + i];
//                 default: return -1; 
//             }
//         }
//         return count;
// 	}
// 	if (SysVars[id].Flags & VF_PROPREAD) {
//   	    return SysVars[id].ReadWrite(ind, -count, (uint32_t*)(buf + ind));
// 	}
// 	return -1;
// }
