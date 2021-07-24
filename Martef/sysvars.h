//  
// Copyright (c) Sasha (Alexander) Chrichoff. All rights reserved.
// Under no circumstances may you delete or modify this file header.
// See LICENSE.TXT file in the project root for full license information.  
//

#define TYPE_NONE     0
#define TYPE_INT8     1
#define TYPE_UINT8    2
#define TYPE_INT16    3
#define TYPE_UINT16   4
#define TYPE_INT32    5
#define TYPE_UINT32   6
#define TYPE_FLOAT    7
const uint8_t TypeSize[8] = {0, 1, 1, 2, 2, 4, 4, 4};

// Conversions to float
inline float FloatFromInt8(void* addr) {return (float)*(int8_t*)addr;}
inline float FloatFromUint8(void* addr) {return (float)*(uint8_t*)addr;}
inline float FloatFromInt16(void* addr) {return (float)*(int16_t*)addr;}
inline float FloatFromUint16(void* addr) {return (float)*(uint16_t*)addr;}
inline float FloatFromInt32(void* addr) {return (float)*(int32_t*)addr;}
inline float FloatFromUint32(void* addr) {return (float)*(uint32_t*)addr;}
inline float FloatFromFloat(void* addr) {return *(float*)addr;}

// Conversions to Int32
inline int32_t Int32FromInt8(void* addr) {return (int32_t)*(int8_t*)addr;}
inline int32_t Int32FromUint8(void* addr) {return (int32_t)*(uint8_t*)addr;}
inline int32_t Int32FromInt16(void* addr) {return (int32_t)*(int16_t*)addr;}
inline int32_t Int32FromUint16(void* addr) {return (int32_t)*(uint16_t*)addr;}
inline int32_t Int32FromInt32(void* addr) {return *(int32_t*)addr;}
inline int32_t Int32FromUint32(void* addr) {return *(int32_t*)addr;}
inline int32_t Int32FromFloat(void* addr) {return (int32_t)*(float*)addr;}

typedef float (*FloatFromAny)(void* addr);
typedef int32_t (*IntFromAny)(void* addr);

typedef int32_t (*VarReadWrite)(uint16_t ind, int16_t count, uint32_t* buf); // returns actual number of transferred if successful  
typedef int32_t* (*VarAddress)(uint16_t ind); // returns variable address  

// Variable Flags
#define VF_TYPE             0x0007          // Mask of type bits
#define VF_DIRECTREAD       0x0010
#define VF_DIRECTWRITE      0x0020
#define VF_PROPREAD         0x0040
#define VF_PROPWRITE        0x0080
#define VF_FLASH            0x1000

struct Vardef {
    uint16_t size;
    uint16_t flags;
    VarAddress addr;
    VarReadWrite read;
    VarReadWrite write;
};

#define nSysVars 334
extern Vardef SysVars[];


int32_t GetSysVar(uint16_t id, uint16_t ind);
int32_t SetSysVar(uint16_t id, uint16_t ind, uint32_t val);
int32_t GetSysArray(uint16_t id, uint16_t ind, uint16_t count, int32_t* buf);
int32_t SetSysArray(uint16_t id, uint16_t ind, uint16_t count, int32_t* buf);
inline Vardef* GetVarDef(uint32_t extid) {
    uint16_t id = *(uint16_t*)&extid, ind = *((uint16_t*)&extid + 1);
    return ((id >= nSysVars) || (ind >= SysVars[id].size)) ? 0 : &SysVars[id];
}
inline uint8_t GetVarType(Vardef* v) {return (uint8_t)(v->flags & VF_TYPE);}
inline uint8_t GetVarTypeSize(Vardef* v) {return TypeSize[GetVarType(v)];}
inline uint8_t GetVarTypeSize(uint8_t type) {return TypeSize[type];}
//inline void* GetVarAddr(Vardef* v, uint16_t ind) {return ((v == 0) || !(v->Flags & VF_DIRECTREAD)) ? 0 : (void*)((int)v->Array+ind*GetVarTypeSize(v));}