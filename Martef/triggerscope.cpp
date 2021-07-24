// cpp
// Scope support
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"
//#include <string.h>
#include <math.h>

//#include "Communication.h"
#include "Global.h"
#include "SysVars.h"
//#include "Martel.h"
#include "TriggerScope.h"

#define VAR_INDEX       104

// // Conversions to float
// float FloatFromInt8(void* addr) {return (float)*(int8_t*)addr;}
// float FloatFromUint8(void* addr) {return (float)*(uint8_t*)addr;}
// float FloatFromInt16(void* addr) {return (float)*(int16_t*)addr;}
// float FloatFromUint16(void* addr) {return (float)*(uint16_t*)addr;}
// float FloatFromInt32(void* addr) {return (float)*(int32_t*)addr;}
// float FloatFromUint32(void* addr) {return (float)*(uint32_t*)addr;}
// float FloatFromFloat(void* addr) {return *(float*)addr;}

// // Conversions to Int32
// inline int32_t Int32FromInt8(void* addr) {return (int32_t)*(int8_t*)addr;}
// inline int32_t Int32FromUint8(void* addr) {return (int32_t)*(uint8_t*)addr;}
// inline int32_t Int32FromInt16(void* addr) {return (int32_t)*(int16_t*)addr;}
// inline int32_t Int32FromUint16(void* addr) {return (int32_t)*(uint16_t*)addr;}
// inline int32_t Int32FromInt32(void* addr) {return *(int32_t*)addr;}
// inline int32_t Int32FromUint32(void* addr) {return *(int32_t*)addr;}
// inline int32_t Int32FromFloat(void* addr) {return (int32_t)*(float*)addr;}

const FloatConvert floatConvert[] = {0, FloatFromInt8, FloatFromUint8, FloatFromInt16, FloatFromUint16, FloatFromInt32, FloatFromUint32, 0}; 
const IntConvert intConvert[] = {0, Int32FromInt8, Int32FromUint8, Int32FromInt16, Int32FromUint16, 0, 0, 0}; 

// Actually stored may be float value
int32_t scBuf[2 * SC_MAXSIGNALS * SC_MAXDEPTH];  // __attribute__ ((aligned (32), __section__(".DmaBuffer")));
struct ScopeConfigStruct ScopeConfig; // __attribute__ ((aligned (32), __section__(".DmaBuffer")));
struct ScopeStruct Scope;

int32_t dummy = 0;

uint16_t ScopeStruct::FillSend(uint8_t buf) {
    struct MultiPacketStruct* send = &Send[buf];
    send->Packets[0].Addr = (uint8_t*)&Config;
    send->Packets[0].Count = 64;
    send->Packets[1].Addr = (uint8_t*)((buf ? scBuf + SC_MAXSIGNALS * SC_MAXDEPTH : scBuf) + Sweep.iStore * Config.nSignals);
    send->Packets[1].Count = 4 * (Config.Depth - Sweep.iStore) * Config.nSignals;
    if (Sweep.iStore != 0) {
        send->Packets[2].Addr = (uint8_t*)(buf ? scBuf + SC_MAXSIGNALS * SC_MAXDEPTH : scBuf);
        send->Packets[2].Count = 4 * Sweep.iStore * Config.nSignals;
    }
    send->Count = (Sweep.iStore == 0) ? 2 : 3;
    SendBuf = buf;
	return 0;
}

void ScopeStruct::Start(int8_t buf) {
	Sweep.pCount = 0; Sweep.sCount = 0; Sweep.iStore = 0; Sweep.Full = 0;
    StoreBuf = buf;
    pStore = StoreBuf ? scBuf + SC_MAXSIGNALS * SC_MAXDEPTH : scBuf;
    Stage = Config.nSignals ? SCS_PREVIEW : SCS_IDLE;
}

static int trace[1000], itrace;
static void Trace(int d) { trace[itrace] = d; if (++itrace >= 1000) itrace = 0; }

void ScopeStruct::Tick() {
	Trace(Stage);
    if ((SendBuf >= 0) && (Send[SendBuf].Count == 0)) SendBuf = -1; 
	if (Stage == SCS_IDLE) {
        return;
    } else if (Stage == SCS_END) {
        if ((SendBuf >= 0) && (Send[SendBuf].Count > 0)) return;
        FillSend(StoreBuf);
        Start(StoreBuf ^ 1);
    } else if (Stage == SCS_TRIGGERED) {
//        if (SendBuf < 0) Stage = SCS_COLLECT;
        Stage = SCS_COLLECT;
    } else if (Stage == SCS_TRIGGER){
        if (Sweep.Full) Stage = SCS_FULL;
    }
	if (Stage < SCS_COLLECT) {	// Triggering
		float t = Trig.Convert ? Trig.Convert(Trig.Var) : *(float*)Trig.Var;
		uint8_t gt = t >= Trig.Level, lt = t <= Trig.Level;
		uint8_t trig = 0;
		if (Stage >= SCS_TRIGGER) {
			if ((Trig.Mode == SCTM_NORMAL) || (Trig.Mode == SCTM_AUTO)) {
				if (Trig.Condition == SCTC_UP) {
					trig = (Trig.Gt >= 0) && (Trig.Gt != gt) && (gt == 1);
				} else if (Trig.Condition == SCTC_DOWN) {
					trig = (Trig.Lt >= 0) && (Trig.Lt != lt) && (lt == 1);
				} else if (Trig.Condition == SCTC_EDGE) {
					trig = ((Trig.Gt >= 0) && (Trig.Gt != gt) && (gt == 1)) || ((Trig.Lt >= 0) && (Trig.Lt != lt) && (lt == 1));
				}
				if (Trig.Mode == SCTM_AUTO) {
					if (++Trig.pCount >= Sweep.Period) {
						Trig.pCount = 0;
						if (++Trig.sCount >= 2 * Sweep.Depth) {
							trig = 1;
						}
					}
				}
			} else if (Trig.Mode == SCTM_FREE) {
				trig = 1;
			}
		}
		if (trig) {
			Stage = SCS_COLLECT;
			Sweep.sCount = Sweep.Preview;
			Trig.pCount = Trig.sCount = 0;
		}
		Trig.Gt = gt; Trig.Lt = lt;
	}
	if (++Sweep.pCount >= Sweep.Period) {
		Sweep.pCount = 0;
		for (uint8_t i = 0; i < Config.nSignals; i++) {
			struct ScopeSigStruct* sig = &Sig[i];
			int32_t v = sig->Convert? sig->Convert(sig->Var) : *(int32_t*)sig->Var;
			*pStore++ = v;
		}
		if (++Sweep.iStore >= Sweep.Depth) {
            Sweep.iStore = 0; Sweep.Full = 1;
            pStore = StoreBuf ? scBuf + SC_MAXSIGNALS * SC_MAXDEPTH : scBuf;
        }
		if (Stage == SCS_PREVIEW) {
			if (Sweep.Full || (Sweep.iStore >= Sweep.Preview)) {
				Stage = SCS_TRIGGER;
			}
		} else if (Stage == SCS_COLLECT) {
			if (++Sweep.sCount >= Sweep.Depth) {
				Stage = SCS_END;
			}
		}
	}
}

void ScopeStruct::Init() {
	MemCpy32(&Config, &ScopeConfig, 16);
    if (Config.nSignals > SC_MAXSIGNALS) Config.nSignals = SC_MAXSIGNALS;
	for (int i = 0; i < Config.nSignals; i++) {
        uint16_t id = *(uint16_t*)&Config.Sig[i];
        if (id < 10000) { // System
            Vardef* v = GetVarDef(Config.Sig[i]);
            Sig[i].Var = (v == 0) ? &dummy : v->addr(*((uint16_t*)&Config.Sig[i]+1));
            Sig[i].Convert = (v == 0) ? 0 : intConvert[GetVarType(v)];
        } else if (id < 20000) { // User Scalar
            id -= 10000;
            // Sig[i].Var = (id < MrtInfo.nGlobals) ? (uint32_t*)(MrtInfo.Globals + id) : &dummy;
            Sig[i].Convert = 0;
        } else {

        }
	}
    uint16_t tr = *(uint16_t*)&Config.Trig;
    if (tr < 10000) { // System
        Vardef* v = GetVarDef(Config.Trig);
        Trig.Var = (v == 0) ? &dummy : v->addr(*((uint16_t*)&Config.Trig+1));
        Trig.Convert = (v == 0) ? 0 : floatConvert[GetVarType(v)];
    } else if (tr < 20000) { // User Scalar
        tr -= 10000;
        // Trig.Var = (tr < MrtInfo.nGlobals) ? (uint32_t*)(MrtInfo.Globals + tr) : &dummy;
        Trig.Convert = (Config.TrigType == TYPE_INT32) ? FloatFromInt32 : 0;
    } else {

    }
	float s = Config.Span * TICKS_IN_SECOND;
    if (s <= SC_MAXDEPTH) {
        Sweep.Period = 1;
        Config.Depth = Sweep.Depth = (uint16_t)ceilf(s);
    } else {
        Sweep.Period = (uint16_t)ceilf(Config.Span / SC_MAXDEPTH * TICKS_IN_SECOND);
        Config.Depth = Sweep.Depth = SC_MAXDEPTH;
    }
    Config.Rate = Sweep.Period * SECONDS_IN_TICK;
    Config.TrigPos = Sweep.Preview = (uint16_t)(Config.Depth * Config.Preview * 0.01);
    Trig.Level = Config.TrigLevel;
	Trig.Mode = Config.TrigMode;
	Trig.Condition = Config.TrigCondition;
	Stage = SCS_IDLE;
	Trig.Gt = -1; Trig.Lt = -1;
    SendBuf = -1;
}

uint8_t ScopeStruct::Configure(uint32_t* from, uint16_t count) {
    MemCpy32((uint8_t*)&ScopeConfig, from, count);
    uint8_t com = ScopeConfig.Command;
    if ((com == SCC_CONFIG) || (com == SCC_CONFIGSTART)) Init();
    switch (com) {
        case SCC_START: case SCC_CONFIGSTART: Start(0); break;
        case SCC_FORCE: Start(0); Stage = SCS_COLLECT; break;
        case SCC_STOP: Stage = SCS_IDLE; break;
    }
	return count;
}

uint8_t ScopeStruct::Read(uint32_t* buf) {
    int8_t i = SendBuf;
    if (i < 0) return 0;
    *buf = (uint32_t)&Send[i].Count;        // Termination flag
    MemCpy32(buf + 1, Send[i].Packets, Send[i].Count << 3);
    return Send[i].Count;
}


