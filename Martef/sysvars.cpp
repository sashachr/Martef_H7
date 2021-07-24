//  
// Copyright (c) Sasha (Alexander) Chrichoff. All rights reserved.
// Under no circumstances may you delete or modify this file header.
// See LICENSE.TXT file in the project root for full license information.  
//

#include "chip.h"
#include <string.h>

#include "Global.h"
#include "Servo.h"
#include "Io.h"
#include "Pwm.h"
#include "Adc.h"
//#include "Dac.h"
//#include "Flash.h"
//#include "Functions.h"
#include "SysTick.h"
#include "TriggerScope.h"
//#include "Martel.h"
#include "Martef.h"
#include "SysVars.h"

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
int32_t GetStatus(uint16_t ind, uint16_t count, uint32_t* buf) {
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
int32_t GetAxisStatus(uint16_t ind, uint16_t count, uint32_t* buf) {
    for (int i=0; i<count; i++) {
        int a = ind + i;
        if (a < N_AXES) {
            *buf++ = 0;
        } else {
            *buf++ = 0;
        }
    }
    return count;
}
int32_t ScopeReadWrite(uint16_t ind, int16_t count, uint32_t* buf) {
    if (count >= 0) {       // Read
        return -Scope.Read(buf);
    } else {   // Write
        return Scope.Configure(buf, -count);
    }
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
// int32_t ReadWriteError(uint16_t ind, int16_t count, uint32_t* buf) {
//     if (count == 1) {
//         *buf = Servo.Error;
//     } else if (count == -1) {
//         Servo.SetError((uint16_t)*buf);
//     } else return -1;
//     return 1;
// }
// int32_t FunctionCall(uint16_t ind, int16_t count, uint32_t* buf) {
//     if (count != -1) return 0;
// 	// return Functions.Function(*buf);
// }
// int32_t WriteIoInversion(uint16_t ind, int16_t count, uint32_t* buf) {
//     if (count != -1) return 0;
//     Io.SetInversion(*buf);
//     return 1;
// }
// int32_t ReadWriteUhrPeriod(uint16_t ind, int16_t count, uint32_t* buf) {
//     if (count == 1) {
//         *(float*)buf = Servo.UhrPeriodRead(); 
//     } else if (count == -1) {
//         Servo.UhrPeriodWrite(*(float*)buf);
//     } else return -1;
//     return 1;
// }
// int32_t ReadWriteUhrDuty(uint16_t ind, int16_t count, uint32_t* buf) {
//     if (count == 1) {
//         *(float*)buf = Servo.UhrRead(); 
//     } else if (count == -1) {
//         Servo.UhrWrite(*(float*)buf);
//     } else return -1;
//     return 1;
// }
// int32_t WriteBreakBeforeMake(uint16_t ind, int16_t count, uint32_t* buf) {Pwm.SetBreakBeforeMake(*(uint8_t*)buf); return 1;}
// int32_t WriteNab4Points(uint16_t ind, int16_t count, uint32_t* buf) {return Pwm.SetTable(PWM_AB4, NPOINTS, ind, -count, buf);}
// int32_t WriteNab5Points(uint16_t ind, int16_t count, uint32_t* buf) {return Pwm.SetTable(PWM_AB5, NPOINTS, ind, -count, buf);}
// int32_t WriteAb4in(uint16_t ind, int16_t count, uint32_t* buf) {return Pwm.SetTable(PWM_AB4, InTable, ind, -count, buf);}
// int32_t WriteAb4aout(uint16_t ind, int16_t count, uint32_t* buf) {return Pwm.SetTable(PWM_AB4, AoutTable, ind, -count, buf);}
// int32_t WriteAb4iout(uint16_t ind, int16_t count, uint32_t* buf) {return Pwm.SetTable(PWM_AB4, IoutTable, ind, -count, buf);}
// int32_t WriteAb4frq(uint16_t ind, int16_t count, uint32_t* buf) {return Pwm.SetTable(PWM_AB4, FreqTable, ind, -count, buf);}
// int32_t WriteAb5in(uint16_t ind, int16_t count, uint32_t* buf) {return Pwm.SetTable(PWM_AB5, InTable, ind, -count, buf);}
// int32_t WriteAb5aout(uint16_t ind, int16_t count, uint32_t* buf) {return Pwm.SetTable(PWM_AB5, AoutTable, ind, -count, buf);}
// int32_t WriteAb5iout(uint16_t ind, int16_t count, uint32_t* buf) {return Pwm.SetTable(PWM_AB5, IoutTable, ind, -count, buf);}
// int32_t WriteAb5frq(uint16_t ind, int16_t count, uint32_t* buf) {return Pwm.SetTable(PWM_AB5, FreqTable, ind, -count, buf);}

// int32_t WriteMonId(uint16_t ind, int16_t count, uint32_t* buf) {return Dac.SetMonId(ind, -count, buf);}
// int32_t WriteMonAddr(uint16_t ind, int16_t count, uint32_t* buf) {return Dac.SetMonAddr(ind, -count, buf);}

//int32_t WriteDout(uint16_t ind, int16_t count, uint32_t* buf) {return Servo.WriteDout(*(float*)buf);}

#define StructDirectRead(struc, var, maxind) \
    [](uint16_t i)->int32_t* {return (int32_t*)&struc[i].var;}, \
    [](uint16_t ind, uint16_t count, uint32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < maxind); i++,j++) *buf++ = struc[j].var; \
        for (; i < count; i++) *buf++ = NONE; \
        return i; \
    }
#define ServoDirectRead(var) StructDirectRead(Servo, var, N_AXES)

#define StructDirectReadWrite(struc, var, maxind) \
    StructDirectRead(struc, var, maxind), \
    [](uint16_t ind, uint16_t count, uint32_t* buf) -> int32_t {  \
        int16_t i = 0, j = ind; \
        for (; (i < count) && (j < maxind); i++,j++) if (!IsNan(*buf)) struc[j].var = *buf++; \
        return i; \
    }
#define ServoDirectReadWrite(var) StructDirectReadWrite(Servo, var, N_AXES)








Vardef SysVars[nSysVars] = {
    {0},
    {4, TYPE_UINT32|VF_PROPREAD, 0, GetStatus, 0},
    {N_AXES, TYPE_UINT32|VF_PROPREAD, 0, GetAxisStatus, 0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0}, // {MaxMrt, TYPE_UINT32|VF_DIRECTREAD|VF_PROPWRITE, (int32_t*)MrtBuf, MrtWrite},
    {0}, // {1, TYPE_UINT32|VF_PROPREAD, 0, MrtValidate},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0}, // {32064, TYPE_UINT32|VF_PROPREAD|VF_PROPWRITE, 0, [](uint16_t ind, int16_t count, uint32_t* buf) -> int32_t {  }}, //ScopeReadWrite},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
   	{0}, // {1, TYPE_UINT32|VF_PROPREAD|VF_PROPWRITE, 0, FlashKey}, 
    {0},
   	{0}, // {1, TYPE_UINT8|VF_PROPREAD|VF_PROPWRITE, 0, FlashSave}, 
    {0},
   	{0}, 
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},     //    {DirectFull(N_AXES, Motions, vel)},
    {0},     //    {DirectFull(N_AXES, Motions, acc)},
    {0},     //    {DirectFull(N_AXES, Motions, dec)},
    {0},     //    {DirectFull(N_AXES, Motions, kDec)},
    {0},     //    {DirectFull(N_AXES, Motions, jerk)},
    {0},
    {0},
    {0},
    {0},
	{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE, ServoDirectRead(RPos), 0}, 
	{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE, ServoDirectRead(RVel), 0}, 
	{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE, ServoDirectReadWrite(RAcc)}, 
	{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE, ServoDirectReadWrite(RJerk)}, 
    {0},    // {DirectFull(N_AXES, Srv, Control.RCur)},
    {0},    // {DirectFull(N_AXES, Srv, Control.TPos)},
    {0},    // {DirectFull(N_AXES, Srv, Control.TVel)},
    {0},
    {0},
    {0},
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD, (int32_t*)&Servo.FPos, 0}, 
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD, (int32_t*)&Servo.FVel, 0}, 
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD, (int32_t*)&Servo.FAcc, 0}, 
    {0},    //   {DirectReadonly(N_AXES, Srv, Control.FCur1)},
    {0},    //   {DirectReadonly(N_AXES, Srv, Control.FCur2)},
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD, (int32_t*)&Servo.Pe, 0}, 
    {0},
    {0},
    {0},
    {0},
    {0},    //    {DirectFull(N_AXES, Srv, Control.KP)},
    {0},
    {0},
    {0},
    {0},
    {0},    //    {DirectFull(N_AXES, Srv, Control.VelL.Pi.KP)},
    {0},    //    {DirectFull(N_AXES, Srv, Control.VelL.Pi.KI)},
    {0},    //    {DirectFull(N_AXES, Srv, Control.VelL.Pi.LI)},
    {0},
    {0},
    {0},    //   {N_AXES, DirectSingleRead(Srv, Control.CurL1.Pi.KP), CurLoopWrite(Pi.KP), CurLopReadWrite(Pi.KP)},
    {0},    //   {N_AXES, DirectSingleRead(Srv, Control.CurL1.Pi.KI), CurLoopWrite(Pi.KI), CurLopReadWrite(Pi.KI)},
    {0},    //   {N_AXES, DirectSingleRead(Srv, Control.CurL1.Pi.LI), CurLoopWrite(Pi.LI), CurLopReadWrite(Pi.LI)},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},  // {1, TYPE_UINT8|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqAc.Enable, 0}, 
    {0},  // {1, TYPE_UINT8|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqAc.Mode, 0}, 
    {0},  // {1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqAc.A1, 0}, 
    {0},  // {1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqAc.A2, 0}, 
    {0},  // {1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqAc.B0, 0}, 
    {0},  // {1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqAc.B1, 0}, 
    {0},  // {1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqAc.B2, 0}, 
    {0},
    {0},
    {0},
    {0},  // 	{1, TYPE_UINT8|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqDc.Enable, 0}, 
    {0},  // 	{1, TYPE_UINT8|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqDc.Mode, 0}, 
    {0},  // 	{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqDc.A1, 0}, 
    {0},  // 	{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqDc.A2, 0}, 
    {0},  // 	{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqDc.B0, 0}, 
    {0},  // 	{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqDc.B1, 0}, 
    {0},  // 	{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.BqDc.B2, 0}, 
    {0},
    {0},
    {0},
    {0},    //   {DirectIntFull(N_AXES, Srv, Control.VelL.Bq1.Mode, int16_t)},
    {0},    //   {DirectIntFull(N_AXES, Srv, Control.VelL.Bq2.Mode, int16_t)},
    {0},    //   {DirectIntFull(N_AXES, Motions, VelocityProfileMode, uint16_t)},
    {0},    //   {DirectIntFull(N_AXES, Srv, UpdateAtIndex, uint16_t)},
    {0},    //   {DirectIntFull(N_AXES, Srv, Control.Simulate, uint16_t)},
    {0},    
    {0},
    {0},
    {0},
    {0},
    {0},    //    {DirectFull(N_AXES, Srv, Control.AFF)},
    {0},    //    {DirectFull(N_AXES, Srv, Control.VFF)},
    {0},
    {0},
    {0},
    {0},    //    {DirectReadonly(N_AXES, Srv, Control.Commut.Phase)},
    {0},
    {0},
    {0},
    {0},
    {0},    //    {DirectFull(N_AXES, Srv, Control.Commut.Offset)},
    {0},    //    {DirectIntFull(N_AXES, Srv, Control.Commut.Offset, int16_t)},
    {0},    //    {DirectFull(N_AXES, Srv, Control.Commut.CountToPhase)},
    {0},    //    {DirectFull(N_AXES, Srv, PhaseAtInd)},
    {0},    //   {DirectFull(N_AXES, Srv, PosAtInd)},
    {0},    //   {DirectFull(N_AXES, Srv, ErrorAtInd)},
    {0},    //   {DirectFull(N_AXES, Srv, Control.DOL)},
    {0},    //   {DirectIntFull(N_AXES, Srv, Control.Motor, uint16_t)},
    {0},
    {0},
	{0}, //{1, TYPE_UINT32|VF_DIRECTREAD, (int32_t*)&Timer.Tick, 0}, 
    {0},    //   {DirectScalarInt(Timer.Counter, uint32_t)},
    {0},    //   {DirectScalarReadonly(Latency)},
    {0},    //   {DirectScalarReadonly(Load)},
    {0},    //    {DirectScalar(MaxLatency)},
    {0},    //    {DirectScalar(MaxLoad)},
    {0},
    {0},
    {0},
    {0},
	{0}, //{1, TYPE_UINT32|VF_DIRECTREAD|VF_DIRECTWRITE, (int32_t*)&Io.Io, 0}, 
	{0}, //{1, TYPE_UINT32|VF_DIRECTREAD, (int32_t*)&Io.IoRaw, 0}, 
	{0}, //{1, TYPE_UINT32|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Io.IoInverse, WriteIoInversion}, 
	{0}, //{1, TYPE_UINT32|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Io.IoDirection, 0}, 
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Encoder.Resolution, 0}, 
    {0},    //    {DirectReadonly(N_AXES, Srv, Control.Enc.IndPos)},
    {0},    //    {DirectReadonly(N_AXES, Srv, Control.RCur)},
    {0},    //    {DirectReadonly(N_AXES, Srv, Control.ROut)},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},    //    {DirectFull(N_AXES, Srv, Control.TL)},
    {0},    //    {DirectFull(N_AXES, Srv, Control.SLP)},
    {0},    //    {DirectFull(N_AXES, Srv, Control.SLN)},
    {0},    //    {DirectFull(N_AXES, Srv, Control.PEL)},
    {0},    //    {DirectFull(N_AXES, Srv, Control.TEL)},
    {0},    //    {DirectFull(N_AXES, Srv, MTL)},
    {0},
    {0},
    {0},
    {0},
    {0},    //   {DirectFull(N_AXES, Srv, TargRad)},
    {0},    //    {DirectFull(N_AXES, Srv, TargSettle)},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
 	{0}, //{1,  TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.DOffs, 0}, 
 	{0}, //{1,  TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.DOL, 0}, 
 	{0}, //{1,  TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.OTL, 0}, 
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
 	{0}, // {2,  TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE, (int32_t*)Dac.Aout, 0}, 
 	{0}, // {2,  TYPE_UINT32|VF_DIRECTREAD|VF_PROPWRITE, (int32_t*)Dac.MonId, WriteMonId}, 
 	{0}, // {10, TYPE_UINT32|VF_DIRECTREAD|VF_PROPWRITE, (int32_t*)Dac.MonAddr, WriteMonAddr}, 
 	{0}, // {2,  TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE, (int32_t*)Dac.MonFact, 0}, 
 	{0}, // {2,  TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE, (int32_t*)Dac.MonOffs, 0}, 
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},    //   {DirectReadonly(N_AXES, Srv, Control.TP1)},
    {0},    //   {DirectReadonly(N_AXES, Srv, Control.TP2)},
    {0},    //   {DirectReadonly(N_AXES, Srv, Control.TP3)},
    {0},    //   {DirectReadonly(N_AXES, Srv, Control.TP4)},
	{0}, // {1,  TYPE_UINT32|VF_PROPWRITE, 0, FunctionCall}, 
	{0}, // {10, TYPE_INT32|VF_DIRECTWRITE, (int32_t*)Functions.Ip, 0}, 
	{0}, // {10, TYPE_FLOAT|VF_DIRECTWRITE, (int32_t*)Functions.Fp, 0}, 
    {0},
    {0},
    {0},
	{0}, //{12, TYPE_FLOAT|VF_DIRECTREAD, (int32_t*)Adc.Ain, 0}, 
	{0}, //{12, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)Adc.AinMin, 0}, 
	{0}, //{12, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)Adc.AinMax, 0}, 
	{0}, //{12, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)Adc.Filter, 0}, 
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD, (int32_t*)&Timer.Late, 0}, 
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD, (int32_t*)&Timer.Use, 0}, 
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE, (int32_t*)&Timer.XLate, 0}, 
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE, (int32_t*)&Timer.XUse, 0}, 
    {0},
    {0},
    {0},
    {0},
    {0},
    {0},
	{0}, //{1, TYPE_UINT8|VF_PROPREAD|VF_PROPWRITE, 0, RwEnable}, 
	{0}, //{1, TYPE_UINT8|VF_PROPREAD|VF_PROPWRITE, 0, RwLinearMode}, 
	{0}, //{1, TYPE_UINT8|VF_PROPREAD|VF_PROPWRITE, 0, RwUhrMode}, 
	{0}, //{1, TYPE_UINT8|VF_PROPREAD|VF_PROPWRITE, 0, RwDcMode}, 
	{0}, //{1, TYPE_UINT8|VF_PROPREAD|VF_PROPWRITE, 0, RwSetOffset}, 
	{0}, //{1, TYPE_UINT8|VF_PROPREAD, 0, RwInTransition}, 
 	{0}, //{1, TYPE_UINT32|VF_DIRECTREAD|VF_DIRECTWRITE, (int32_t*)Servo.Mode, 0}, 
    {0},
    {0},
    {0},
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE, (int32_t*)&Servo.In, 0}, 
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD|VF_PROPWRITE, (int32_t*)&Servo.Out, WriteDout}, 
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.NormalOffset, 0}, 
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.LinearOffset, 0}, 
	{0}, //{1, TYPE_FLOAT|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.DcOffset, 0},
    {0},
    {0},
	{0}, //{1, TYPE_FLOAT|VF_PROPREAD|VF_PROPWRITE|VF_FLASH, 0, ReadWriteUhrPeriod}, 
	{0}, //{1, TYPE_FLOAT|VF_PROPREAD|VF_PROPWRITE|VF_FLASH, 0, ReadWriteUhrDuty}, 
	{0}, //{1, TYPE_UINT8|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Pwm.BreakBeforeMake, WriteBreakBeforeMake}, 
	{0}, //{1, TYPE_UINT32|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Pwm.Ab4.Npoints, WriteNab4Points}, 
	{0}, //{20, TYPE_FLOAT|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Pwm.Ab4.In, WriteAb4in}, 
	{0}, //{20, TYPE_FLOAT|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Pwm.Ab4.Aout, WriteAb4aout}, 
    {0}, //{20, TYPE_FLOAT|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Pwm.Ab4.Iout, WriteAb4iout},
    {0}, //{20, TYPE_FLOAT|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Pwm.Ab4.Freq, WriteAb4frq},
	{0}, //{1, TYPE_UINT32|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Pwm.Ab5.Npoints, WriteNab5Points}, 
	{0}, //{20, TYPE_FLOAT|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Pwm.Ab5.In, WriteAb5in}, 
	{0}, //{20, TYPE_FLOAT|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Pwm.Ab5.Aout, WriteAb5aout}, 
    {0}, //{20, TYPE_FLOAT|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Pwm.Ab5.Iout, WriteAb5iout},
    {0}, //{20, TYPE_FLOAT|VF_DIRECTREAD|VF_PROPWRITE|VF_FLASH, (int32_t*)&Pwm.Ab5.Freq, WriteAb5frq},
	{0}, //{1, TYPE_UINT32|VF_DIRECTREAD, (int32_t*)&Servo.Safety, 0}, 
	{0}, //{1, TYPE_UINT32|VF_DIRECTREAD, (int32_t*)&Servo.SafetyRaw, 0}, 
	{0}, //{1, TYPE_UINT32|VF_DIRECTREAD|VF_DIRECTWRITE|VF_FLASH, (int32_t*)&Servo.SafetyMask, 0}, 
	{0}, //{1, TYPE_UINT16|VF_PROPREAD|VF_PROPWRITE, 0, ReadWriteError}, 
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