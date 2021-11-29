// Motion.cpp
// Motion calculation / generation
// COPYRIGHT 2012 Sasha Chrichov

#include "chip.h"     // Chip definition

#include <stdint.h>
#include <math.h>
#include <new>

#include "global.h"     // DSP280x Headerfile Include File
#include "servo.h"
#include "motion.h"

MotionWrap Motions;

// Additional mathematics
float atan2f(float x, float y) {
	if (fabsf(y) <= fabsf(x)) {
		float a = atanf(y / x);
		return (x > 0)? a : (y > 0)? a + F_PI : a - F_PI;
	} else {
		float b = -atanf(x / y);
		return (y > 0)? b + (F_PI * 0.5F) : b - (F_PI * 0.5F);
	}
}
	
#define VPM_TRAPEZOIDAL				1
#define VPM_TRAPEZOIDALORTABLE		2
#define VPM_MINIMUMENERGY			3
#define VPM_MINIMUMENERGYORTABLE	4
#define VPM_THIRDORDER				5
struct TableMotionDef TableMotions[1];

// union MaxMotion {
// 	TrapezoidalMotion m0;
// 	MinimumEnergyMotion m1;
// 	ThirdOrderMotion m2;
// 	TableMotion m3;
// 	Line2Motion m4;
// 	Arc2Motion m5;
// 	TBlendedMotion m6;
// } mMotion[NAX];

// MotionBase* Motion[NAX];				// Should be the greatest class of the family

// Third-order motion profile with jerk
float CubicRootTable[113];

void FillCubicRootTable() {
	for (int i = 16; i <= 128; i++) {
		double x = 8.0 / 128 * i;
		double v = pow(x, 1.0 / 3);
		*(uint32_t*)&v &= 0xE0000000;       // Truncate to prevent rounding
		CubicRootTable[i - 16] = (float)v;
	}
}

float FastCubicRoot(float x) {
	uint8_t negative = x < 0;
	if (negative) x = -x;
	int32_t exp = (((int16_t*)&x)[1] >> 7) - 127;
	int32_t exp3 = exp / 3;
	int32_t rem = exp - exp3 * 3;
	if (rem < 0) {exp3--; rem += 3;}
	uint32_t mantissa = (*(uint32_t*)&x & 0x007FFFFF) | 0x00800000;
	mantissa >>= 2 - rem;
	int segm = (mantissa >> 17) - 16;
	float frem = (mantissa & 0x0001FFFF) * 0.00000762939453125F;  // / 131072.0F;
	float res = (1 - frem) * CubicRootTable[segm] + frem * CubicRootTable[segm + 1];
	((uint16_t*)&res)[1] += exp3 << 7;  //res *= (float)Math.Pow(2, re);
	return negative? -res : res;
}

void MotionBase::Init(int i) {
	Index = i;
	time = 0; phase = 0; 
	MVel = 10; MAcc = 100; MJerk = 10000;
	MTv = 500; MTa = 50; MTj = 10;
	SetType(M_DEFAULT);
}
void MotionBase::SetType(int32_t type) {
	Type = type;
	switch (type) {
		case M_TRAPEZOIDAL: new(this) TrapezoidalMotion(); break;
		case M_THIRDORDER: new(this) ThirdOrderMotion(); break;
		case M_TBLENDED: new(this) TBlendedMotion(); break;
		default: new(this) MotionBase();
	} 
}
// Kill motion
void MotionBase::Kill() {
	new(this) TrapezoidalMotion((uint16_t)M_TRAPEZOIDAL);
	this->Kill();
}

void MotionBase::Stop() {
	new(this) MotionBase();
}

void TrapezoidalMotion::Calculate(float p0, float v0, float p1, float v1) {
    P0 = p0; V0 = v0; P1 = p1; V1 = v1; 
	D = P1 - P0;
	float S0 = v0 * fabsf(v0) / MAcc * 0.5F;
	float VT = (S0 > D)? -MVel : MVel;
	float A1 = (v0 > VT)? -MAcc : MAcc;
	float A3 = (VT > 0.0F)? -MAcc : MAcc;
	float T1 = (VT - v0) / A1;
	float T3 = -VT / A3;
	float S1 = (v0 + VT) * 0.5F * T1;
	float S3 = VT * 0.5F * T3;
	float S2 = D - S1 - S3;
	float T2;
	if ((S2==0) || ((S2>0)==(VT>0))) {
		T2 = S2/VT;
	} else {
		float VT1 = v0 * v0 * 0.5F + A1 * D;
		VT1 = (VT1 > 0)? sqrtf(VT1) : 0;
		VT = (VT > 0)? VT1 : -VT1;
		T1 = (VT - v0)/A1;
		T2 = 0.0F;
		T3 = -VT / A3;
		S1 = (v0 + VT) * 0.5F * T1;
		S2 = 0;
	}
	Seg[0].StartTime = 0.0F;
	Seg[0].Duration = Seg[1].StartTime = T1;
	Seg[1].Duration = T2; Seg[2].StartTime = T1+T2;
	Seg[2].Duration = T3;
	Seg[0].Acc = A1; Seg[1].Acc = 0.0F; Seg[2].Acc = A3;
	Seg[0].Vel = v0; Seg[1].Vel = Seg[2].Vel = VT;
	Seg[0].Pos = p0; Seg[1].Pos = p0+S1; Seg[2].Pos = Seg[1].Pos+S2;
	RPos = Seg[0].Pos; RVel = Seg[0].Vel; RAcc = Seg[0].Acc; RJerk = 0;
	CurSegment = &Seg[0];
	phase = 1;
	time = 0;
	Servo->StartMotion();
}

// TrapezoidalMotion::TrapezoidalMotion(float p1) : MotionBase (Motion[0].MVel, Motion[0].MAcc, Motion[0].MAcc, Motion[0].MAcc) {
// 	type = M_TRAPEZOIDAL;
// 	P0 = 0; P1 = p1; V0 = 0; V1 = 0;
// }

void TrapezoidalMotion::Tick() {
	if (Servo->TPosChanged()) {
		if (Servo->ValidatePositionLoop()) {
			Servo->GroupGetTPos(TPos);
			Calculate(Servo->RPos, Servo->RVel, TPos[0], 0);
		}
	}
	if (phase != 0) {
		time += SECONDS_IN_TICK;
		while (1) {
			float T = time - CurSegment->StartTime;
			if (T < CurSegment->Duration) {
				float V = CurSegment->Acc*T;
				RVel = CurSegment->Vel + V;
				RPos = CurSegment->Pos + (CurSegment->Vel + V/2) * T;
				break;
			} else {
				if (phase<=2) {
					phase++;
					CurSegment++;
					RAcc = CurSegment->Acc;
				} else {	// Motion finished
					RPos = P1;
					RVel = RAcc = 0;
					phase = 0;
					Servo->EndMotion();
					break;
				}
			}
		}
	} else {
	}
}

// Kill motion
void TrapezoidalMotion::Kill() {
	// Seg[2].Acc = (RVel >= 0)? -KDec : KDec;
	// Seg[2].Duration = -RVel / Seg[2].Acc;
	// Seg[0].StartTime = Seg[0].Duration = Seg[1].StartTime = Seg[1].Duration = Seg[2].StartTime = 0;
	// Seg[2].Pos = RPos;
	// Seg[2].Vel = RVel;
	// P1 = Seg[2].Pos + Seg[2].Vel * Seg[2].Duration / 2;
	// RAcc = Seg[2].Acc;
	// CurSegment = &Seg[2];
	// time = 0;
	// phase = 3;
}

ThirdOrderMotion::ThirdOrderMotion() : MotionBase () {
	P0 = Servo->RPos; P1 = Servo->TPos; D = P1 - P0;
    uint8_t negative = D < 0;
    if (negative) D = -D;
    float J = MJerk, A = MAcc, V = MVel;
    float TJ = A / J, VJ = J * TJ * TJ;
    //float aV = V, aA = A;
    float TV, TA, TAJ, DAJ;
    float v = V;
    if (V >= VJ) {
	    TA = (V - VJ) / A;
	    TAJ = TA + 2.0F * TJ;
	} else {
	    TA = 0;
	    TJ = sqrtf(V / J);
	    TAJ = 2.0F * TJ;
    }
    DAJ = V * TAJ;
    if (D >= DAJ) {
	    TV = (D - DAJ) / V;
	} else {
	    if (TA > 0) {
		    v = A * A / J * 0.5F;
		    v = -v + sqrtf(v * v + D * A);
		    TA = (v > VJ)? (v - VJ) / A : 0;
	    }
	    if (TA <= 0) {
		    TA = 0;
		    v = FastCubicRoot(D * D * J * 0.25F);
		    TJ = sqrtf(v / J);
	    }
	    TAJ = TA + 2 * TJ;
	    DAJ = v * TAJ;
	    TV = (D > DAJ)? (D - DAJ) / v : 0;
    }
    Seg[0].StartTime = 0;
    Seg[0].Duration = TJ;
    Seg[0].Jerk = negative? -J : J;
    Seg[0].Acc = 0;
    Seg[0].Vel = 0;
    Seg[0].Pos = P0;
    Seg[1].StartTime = Seg[0].StartTime + Seg[0].Duration;
    Seg[1].Duration = TA;
    Seg[1].Jerk = 0;
    Seg[1].Acc = Seg[0].Jerk * TJ;
    Seg[1].Vel = Seg[1].Acc * TJ * 0.5F;
    Seg[1].Pos = P0 + Seg[1].Vel * TJ * 0.333333333F;
    Seg[2].StartTime = Seg[1].StartTime + Seg[1].Duration;
    Seg[2].Duration = TJ;
    Seg[2].Jerk = -Seg[0].Jerk;
    Seg[2].Acc = Seg[1].Acc;
    Seg[2].Vel = (TA > 0)? Seg[1].Vel + Seg[1].Acc * TA : Seg[1].Vel;
    Seg[2].Pos = (TA > 0)? Seg[1].Pos + Seg[1].Vel * TA + Seg[1].Acc * TA * TA * 0.5F : Seg[1].Pos;
    Seg[3].StartTime = Seg[2].StartTime + Seg[2].Duration;
    Seg[3].Duration = TV;
    Seg[3].Jerk = 0;
    Seg[3].Acc = 0;
    Seg[3].Vel = negative? -v : v;
    Seg[3].Pos = P0 + (negative? -DAJ : DAJ) * 0.5F;
    Seg[4].StartTime = Seg[3].StartTime + Seg[3].Duration;
    Seg[4].Duration = TJ;
    Seg[4].Jerk = Seg[2].Jerk;
    Seg[4].Acc = 0;
    Seg[4].Vel = Seg[3].Vel;
    Seg[4].Pos = P1 + P0 - Seg[3].Pos;
    Seg[5].StartTime = Seg[4].StartTime + Seg[4].Duration;
    Seg[5].Duration = TA;
    Seg[5].Jerk = 0;
    Seg[5].Acc = -Seg[2].Acc;
    Seg[5].Vel = Seg[2].Vel;
    Seg[5].Pos = P1 + P0 - Seg[2].Pos;
    Seg[6].StartTime = Seg[5].StartTime + Seg[5].Duration;
    Seg[6].Duration = TJ;
    Seg[6].Jerk = Seg[0].Jerk;;
    Seg[6].Acc = -Seg[1].Acc;
    Seg[6].Vel = Seg[1].Vel;
    Seg[6].Pos = P1 + P0 - Seg[1].Pos;
	RJerk = Seg[0].Jerk;
	CurSegment = &Seg[0];
	phase = 1;
}

void ThirdOrderMotion::Tick() {
	//SCB_CleanInvalidateDCache();
	//SCB_DisableDCache();
	if (phase != 0) {
		time += SECONDS_IN_TICK;
		while (1) {
			float T = time - CurSegment->StartTime;
			if (T < CurSegment->Duration) {
				float dA = CurSegment->Jerk * T;
				float dV = CurSegment->Acc * T;
				float dV1 = dA * T * 0.5F;
				RAcc = CurSegment->Acc + dA;
				RVel = CurSegment->Vel + dV + dV1;
				RPos = CurSegment->Pos + (CurSegment->Vel + dV * 0.5F + dV1 * 0.333333333F) * T;
				break;
			} else {
				if (phase < 7) {
					phase++;
					CurSegment++;
					RJerk = CurSegment->Jerk;
				} else {	// Motion finished
					RPos = P1;
					RVel = RAcc = RJerk = 0;
					new(this) MotionBase();
					break;
				}
			}
		}
	} else {
	}
	//SCB_EnableDCache();
}

TableMotion::TableMotion() : MotionBase() {
	P0 = Servo->RPos; V0 = Servo->RVel; P1 = Servo->TPos; V1 = 0; 
	D = P1 - P0;
	total = (uint16_t)(TableMotions[0].Duration*TICKS_IN_MILLISECOND + 0.5F);
	factor = D / TableMotions[0].Distance;
	current = 0;
	point = TableMotions[0].Points;
	phase = 12;
}

void TableMotion::Tick() {
	if (current < total) {
		RPos = factor * point[0];
		RVel = factor * point[1];
		RAcc = factor * point[2];
		point += 3;
		current++;
	} else {
		RPos = P1;
		RVel = RAcc = 0;
		new(this) MotionBase();
	}
}

MinimumEnergyMotion::MinimumEnergyMotion(float p1, float v1) : TrapezoidalMotion((uint16_t)M_MINIMUMENERGY) {
	P0 = RPos; V0 = RVel; P1 = p1; V1 = v1;
	D = P1 - P0;
	float T1 = 2 * MVel / MAcc;
	float PF = MAcc * T1 * T1 * 4 * 0.166666666F;
	float d = fabsf(D);
	float T2, VM;
	float A1 = (D > 0)? MAcc : -MAcc;
	if (d >= PF) {
		T2 = (d - PF) / MVel;
		VM = (D > 0)? MVel : -MVel;
	} else {
		T2 = 0;
		T1 = sqrtf(1.5F * d / MAcc);
		VM = A1 * T1 * 0.5F;
	}
	float J = -A1 / T1;
	Seg[0].StartTime = 0.0F;
	Seg[0].Duration = Seg[2].Duration = Seg[1].StartTime = T1;
	Seg[1].Duration = T2; Seg[2].StartTime = T1+T2;
	Seg[0].Acc = Seg[2].Acc = J; Seg[1].Acc = 0.0F;
	Seg[0].Vel = 0; Seg[1].Vel = Seg[2].Vel = VM;
	Seg[0].Pos = P0;
	Seg[1].Pos = P0 + (A1 * 0.5F + J * T1 * 0.166666666F) * T1 * T1;
	Seg[2].Pos = P1 - (VM + J * T1 * T1 * 0.166666666F) * T1;
	CurSegment = &Seg[0];
	phase = 1;	
}

void MinimumEnergyMotion::Tick() {
	if (phase != 0) {
		time += SECONDS_IN_TICK;
		float T;
		while (1) {
			T = time - CurSegment->StartTime;
			if (T < CurSegment->Duration) break;
			if (phase<=2) {
				phase++;
				CurSegment++;
			} else {	// Motion finished
				RPos = P1;
				RVel = RAcc = 0;
				new(this) MotionBase();
				return;
			}
		}
		if (phase == 1) {
			float acc = (CurSegment->Acc < 0)? MAcc : -MAcc;
			RAcc = acc + CurSegment->Acc * T;
			RVel = (acc + CurSegment->Acc * T * 0.5F) * T;
			RPos = CurSegment->Pos + (acc * 0.5F + CurSegment->Acc * T * 0.166666666F) * T * T;
		} else if (phase == 2) {
			RAcc = 0;
			RVel = CurSegment->Vel;
			RPos = CurSegment->Pos + CurSegment->Vel * T;
		} else {
			RAcc = CurSegment->Acc * T;
			RVel = CurSegment->Vel + CurSegment->Acc * T * T * 0.5F;
			RPos = CurSegment->Pos + (CurSegment->Vel + CurSegment->Acc * T * T * 0.166666666F) * T;
		}
	}
}

void StartOneAxisMotion(MotionBase* M, float p1, float v1) {
// //	SCB_CleanInvalidateDCache();
// //	SCB_DisableDCache();
// 	if ((M->RPos == p1) && (M->RVel == v1)) {new(M) MotionBase(M_NONE); return;}
// 	float D = p1 - M->RPos, TD = TableMotion[0].Distance;
// 	if (((M->VelocityProfileMode == VPM_TRAPEZOIDALORTABLE) || (M->VelocityProfileMode == VPM_MINIMUMENERGYORTABLE)) &&
// 	(TD != 0) && (M->RVel == 0) && (fabsf(fabsf(D) - TD) <= TableMotion[0].Tolerance)) {
// 		new(M) TableMotion(p1, v1);
// 	} else if (((M->VelocityProfileMode == VPM_MINIMUMENERGY) || (M->VelocityProfileMode == VPM_MINIMUMENERGYORTABLE)) &&	(M->RVel == 0)) {
// 		new(M) MinimumEnergyMotion(p1, v1);
// 	} else if ((M->VelocityProfileMode == VPM_THIRDORDER) && (M->RVel == 0)) {
// 		new(M) ThirdOrderMotion(p1);
// 	} else {
// 		new(M) TrapezoidalMotion(p1, v1);
// 	}
// //	SCB_EnableDCache();
}

Line2Motion::Line2Motion(float t0, float t1) : MotionBase() {
	s0 = Motions[0].RPos; s1 = Motions[1].RPos; f0 = t0; f1 = t1;
	float d0 = f0 - s0, d1 = f1 - s1;
	L = sqrtf(d0 * d0 + d1 * d1); k0 = d0 / L; k1 = d1 / L;
	new(&m) TrapezoidalMotion(L);
	new(&m) TrapezoidalMotion(L, 0);
	Motions[0].phase = m.phase; Motions[1].phase = 21;	// dependent
}

void Line2Motion::Tick() {
	m.Tick();
	if (m.phase > 0) {
		Motions[0].RPos = s0 + m.RPos * k0; Motions[1].RPos = s1 + m.RPos * k1;
		Motions[0].RVel = m.RVel * k0; Motions[1].RVel = m.RVel * k1;
		Motions[0].RAcc = m.RAcc * k0; Motions[1].RAcc = m.RAcc * k1;
		Motions[0].time = m.time; Motions[0].phase = m.phase;
		} else {
		Motions[0].RPos = f0; Motions[1].RPos = f1;
		Motions[0].RVel = 0; Motions[1].RVel = 0;
		Motions[0].RAcc = 0; Motions[1].RAcc = 0;
		Motions[0].time = 0; Motions[0].phase = 0;
		new(this) MotionBase();
	}
}

void Line2Motion::Kill() {
	m.Kill();
}

void StartLine2Motion(float t0, float t1) {
	new(&Motions[1]) MotionBase();
	new(&Motions[0]) Line2Motion(t0, t1);
	Servo[0].TPos = t0; Servo[1].TPos = t1;
	Servo[0].SetMotionState(1); Servo[1].SetMotionState(1);
}

Arc2Motion::Arc2Motion(float t0, float t1, float ce0, float ce1, int d) : MotionBase() {
	//SCB_CleanInvalidateDCache();
	//SCB_DisableDCache();
	dir = (int8_t)d;
	s0 = Motions[0].RPos; s1 = Motions[1].RPos; f0 = t0; f1 = t1; c0 = ce0; c1 = ce1;
	float sv0 = s0 - c0, sv1 = s1 - c1, fv0 = f0 - c0, fv1 = f1 - c1;
	sa = atan2f(sv0, sv1); fa = atan2f(fv0, fv1);
	float a = fa - sa;
	if ((a != 0) && ((a > 0) != (dir > 0))) a += (a > 0)? -(2 * F_PI) : (2 * F_PI);
	sr = sqrtf(sv0 * sv0 + sv1 * sv1); fr = sqrtf(fv0 * fv0 + fv1 * fv1); r = (sr + fr) * 0.5;
	L = fabsf(r * a); ka = a / L; kr = (fr - sr) / L;
	new(&m) TrapezoidalMotion(L);
	new(&m) TrapezoidalMotion(L, 0);
	Motions[0].phase = m.phase; Motions[1].phase = 21;	// dependent
}

void Arc2Motion::Tick() {
	m.Tick();
	if (m.phase > 0) {
		float a = sa + m.RPos * ka, r = sr + m.RPos * kr;
		float s = sinf(a), c = cosf(a), ss = s, sc = c;
		if (dir < 0) {ss = -ss; sc = -sc;} 
		Motions[0].RPos = c0 + r * c; Motions[1].RPos = c1 + r * s;
		Motions[0].RVel = m.RVel * ss; Motions[1].RVel = m.RVel * sc;
		Motions[0].RAcc = - m.RVel * m.RVel * c + m.RAcc * ss; Motions[1].RAcc = - m.RVel * m.RVel * s + m.RAcc * sc;
		Motions[0].time = m.time; Motions[0].phase = m.phase;
	} else {
		Motions[0].RPos = f0; Motions[1].RPos = f1;
		Motions[0].RVel = 0; Motions[1].RVel = 0;
		Motions[0].RAcc = 0; Motions[1].RAcc = 0;
		Motions[0].time = 0; Motions[0].phase = 0;
		new(this) MotionBase();
	}
}

void Arc2Motion::Kill() {
	m.Kill();
}

void StartArc2Motion(float t0, float t1, float c0, float c1, int d) {
	new(&Motions[1]) MotionBase();
	new(&Motions[0]) Arc2Motion(t0, t1, c0, c1, d);
	Servo[0].TPos = t0; Servo[1].TPos = t1;
	Servo[0].SetMotionState(1); Servo[1].SetMotionState(1);
}

void TBlendedMotion::Tick() {
	uint8_t newmotion = 0;
//	for (uint8_t i = 0; i < nax; i++) {
//		uint8_t j = Servo->Giax[i];
//	}
	if (newmotion) {

	}
}

// MotionBase* DefaultMotion(int i) {
// 	return new(&Motions[i]) TrapezoidalMotion();
// }

void MotionInit() {
	FillCubicRootTable();
	for (int i = 0; i < NAX; i++) { 
		Motions[i].Init(i); 
	}
}
