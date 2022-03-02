// Motion.h
// Motion calculation / generation
// COPYRIGHT 2012 Sasha Chrichov

#pragma once

// Motions
#define M_NONE			0
#define M_DEPENDENT		1
#define M_TRAPEZOIDAL	2
#define M_TABLE			3
#define M_MINIMUMENERGY	4
#define M_THIRDORDER	5
#define M_LINE2			100
#define M_ARC2			101
#define M_TBLENDED		103
#define M_DEFAULT		M_TRAPEZOIDAL

// Joining
#define J_COMPLETE		1
#define J_BLENDING		2
#define J_IMMEDIATE		3

#define MOTION_TABLE_MAX	1 // 1500

class ServoStruct;

class MotionBase {
public:
	uint8_t Index;				// Index in Motion array
	int32_t Type;
	int32_t Join;
	float TPos[NAX];					// Target position
	float MVel, MAcc, MJerk; 	// Motion parameters
	float MTv, MTa, MTj;		// Parameters for time-based motion
	float GPos, GVel, GAcc, GJerk;
	float time;
	uint16_t phase;
	ServoStruct* Servo;
	MotionBase() {} 
	void Init(int i);
	float Euclid(float* p0, float* p1, int n) {
		float s = 0;
		for (int i  = 0; i < n; i++) { float d = *p1++ - *p0++; s += d * d;}
		return sqrtf(s);
	}
	void SetType(int32_t type);
	virtual void Tick() {}
	virtual void Kill();
	virtual void Stop(); 
	virtual void Next() {}
};

struct TrapezoidalMotionSegment {
	float StartTime;
	float Duration;
	float Acc, Vel, Pos;		// Initial values
};

class TrapezoidalMotion : public MotionBase {
public:
	float P0, P1, V0, V1;
	float D;
	struct TrapezoidalMotionSegment Seg[3];
	struct TrapezoidalMotionSegment *CurSegment;
	TrapezoidalMotion(float p1, float v1);
	TrapezoidalMotion(uint16_t type) : MotionBase() {}
	TrapezoidalMotion() {}
	TrapezoidalMotion(float p1);
	void Calculate(float p0, float v0, float p1, float v1, float vel, float acc);
	virtual void Tick();
	virtual void Kill();
};

class GroupTrapezoidalMotion : public MotionBase {
public:
	float p0[NAX], p1[NAX], c[NAX];
	float s;
	struct TrapezoidalMotionSegment Seg[3];
	struct TrapezoidalMotionSegment *CurSegment;
	GroupTrapezoidalMotion() {}
	void Calculate(float s, float vel, float acc);
	virtual void Tick();
	virtual void Kill();
};

class MinimumEnergyMotion : public TrapezoidalMotion {
public:
	MinimumEnergyMotion() {}
	MinimumEnergyMotion(float p1, float v1);
	virtual void Tick();
};

struct ThirdOrderMotionSegment {
	float StartTime;
	float Duration;
	float Jerk, Acc, Vel, Pos;		// Initial values
};

class ThirdOrderMotion : public MotionBase {
	public:
	float P0, P1;
	float D;
	struct ThirdOrderMotionSegment Seg[7];
	struct ThirdOrderMotionSegment *CurSegment;
	ThirdOrderMotion();
	virtual void Tick();
};

struct TableMotionPoint {
	float Pos, Vel, Acc;
};
struct TableMotionDef {
	uint32_t Version;
	float Tolerance;
	float Distance;
	float Duration;
	float Points[MOTION_TABLE_MAX];
};

class TableMotion : public MotionBase {
public:
	float P0, P1, V0, V1;
	float D;
	uint16_t total, current;
	float factor;
	float *point;
	TableMotion();
	virtual void Tick();
};

class Line2Motion : public MotionBase {
public:
	float s0, s1, f0, f1, L, k0, k1;
	TrapezoidalMotion m;
	Line2Motion() {}
	Line2Motion(float t0, float t1);
	virtual void Tick();
	virtual void Kill();
};

class Arc2Motion : public MotionBase {
public:
	float s0, s1, f0, f1, c0, c1, sr, fr, r, L, sa, fa, ka, kr;
	int8_t dir;
	TrapezoidalMotion m;
	Arc2Motion() {}
	Arc2Motion(float t0, float t1, float c0, float c1, int d);
	virtual void Tick();
	virtual void Kill();
};

#define TB_FIFO		3
class TimeBased : public MotionBase {		// Time-bases blended motion 
public:
	int npoint;
	float pt[TB_FIFO][NAX+3];		// Start, intermediate and final points
	float p[NAX], v[NAX], a[NAX], j[NAX], t; 
	float vm[NAX], am[NAX];
	int phase;
	TimeBased() { }
	virtual void Tick();
//	virtual void Kill();
};

void StartOneAxisMotion(MotionBase* M, float p1, float v1);

void StartLine2Motion(float t0, float t1);

void StartArc2Motion(float t0, float t1, float c0, float c1, int d);

//void MotionTick(MotionStruct& M);

//MotionBase* DefaultMotion(int i);

class MotionWrap {
public:
	union {
		char m0[sizeof(MotionBase)];
		char m1[sizeof(TrapezoidalMotion)];
		char m2[sizeof(MinimumEnergyMotion)];
		char m3[sizeof(ThirdOrderMotion)];
		char m4[sizeof(TableMotion)];
		char m5[sizeof(Line2Motion)];
		char m6[sizeof(Arc2Motion)];
		char m7[sizeof(TimeBased)];
	} MotionContainers[NAX];
	MotionBase& operator [] (int i) { return *(MotionBase*)&MotionContainers[i]; }
};
extern MotionWrap Motions;

void MotionInit();

