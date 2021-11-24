// Motion.h
// Motion calculation / generation
// COPYRIGHT 2012 Sasha Chrichov

#pragma once

#define M_NONE			0
#define M_TRAPEZOIDAL	1
#define M_TABLE			2
#define M_MINIMUMENERGY	3
#define M_THIRDORDER	4
#define M_LINE2			100
#define M_ARC2			101

#define MOTION_TABLE_MAX	1 // 1500

class MotionBase {
public:
	uint32_t ax;				// Bitwise specification of related axes
	uint8_t iax[NAX];			// Related axes
	uint8_t nax;				// Number of related axes
	uint8_t index;				// Index in Motion array
	uint8_t group;				// Index of the group
	float TPos;					// Target position
	float MVel, MAcc, MJerk; 	// Motion parameters
	float MTv, MTa, MTj;		// Parameters for time-based motion
	float RPos, RVel, RAcc, RJerk;
	float time;
	uint16_t phase;
	uint16_t type;
	uint16_t VelocityProfileMode;
	MotionBase() {type = M_NONE;} 
	MotionBase(uint16_t Type) {type = Type;}
	void Init(int i);
	void Reset();
	void Group(int32_t gr);
	virtual void Tick() {}
	virtual void Kill();
	virtual void Stop(); 
	virtual void Next() {}

	float tpos;	
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
	TrapezoidalMotion(uint16_t type) : MotionBase(type) {}
	TrapezoidalMotion() {}
	TrapezoidalMotion(float p1);
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
	ThirdOrderMotion(float p1);
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
	TableMotion(float p1, float v1);
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

class TBlended : public MotionBase {		// Time-bases blended motion 
public:
	float tp0[NAX], tp1[NAX], tp2[NAX];		// Start, intermediate and final points
	float p1, p2;		// Segments lengths
	TrapezoidalMotion m;
	TBlended() {}
	virtual void Tick();
	virtual void Kill();
};

extern struct TableMotionDef TableMotions[1];
extern Arc2Motion Motion[NAX];

void StartOneAxisMotion(MotionBase* M, float p1, float v1);

void StartLine2Motion(float t0, float t1);

void StartArc2Motion(float t0, float t1, float c0, float c1, int d);

//void MotionTick(MotionStruct& M);

MotionBase* DefaultMotion(int i);

void MotionInit();
inline void MotionTick() { for (int i = 0; i < NAX; i++) Motion[i].Tick(); }


