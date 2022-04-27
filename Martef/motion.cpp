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
// 	TimeBased m6;
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
	T1 = 0.05F; T2 = 0.5F; Com = 10;
	SetType(M_DEFAULT);
}
void MotionBase::SetType(int32_t type) {
	Type = type;
	switch (type) {
		case M_COMMUTATION: new(this) Commutation(); break;
		case M_TRAPEZOIDAL: new(this) GroupTrapezoidalMotion(); break;
		case M_THIRDORDER: new(this) ThirdOrderMotion(); break;
		case M_BLENDED: new(this) Multipoint(); break;
		default: new(this) MotionBase();
	} 
}
void MotionBase::Disable() {
	Servo->FifoFlush();
	phase = 0;
}

void Commutation::Tick() {
	if (phase == 20) {
		t0 = 0;
		Servo->CdOut = Servo->CqOut = 0;
		Servo->Teta = M_PI_4;
		Servo->RState = (Servo->RState & ~(SM_POSITIONLOOP|SM_VELOCITYLOOP|SM_CURRENTLOOP|SM_COMMUTATION)) | (SM_MOTION|SM_PWM|SM_ENABLE);
		phase = 21;
	} else if ((phase == 21) || (phase == 25)) {
		t0 += SECONDS_IN_TICK;
		if (t0 < T1) {
			Servo->CqOut = Minf(Com, 100) * t0 / T1;
		} else {
			Servo->CqOut = Minf(Com, 100);
			t0 -= T1;
			phase++;
		}
	} else if ((phase == 22) || (phase == 26)) {
		t0 += SECONDS_IN_TICK;
		if (t0 < T2) {
		} else {
			t0 -= T2;
			if (phase == 24) Servo->RState |= ~SM_ENABLE;
			phase++;
		}
	} else if ((phase == 23) || (phase == 27)) {
		Servo->CdOut = Servo->CqOut = 0;
		t0 = 0;
		phase++;
	} else if (phase == 24) {
		Servo->Teta = 0;
		t0 = 0;
		phase++;
	} else if (phase == 28) {
		Servo->RState &= ~SM_ENABLE;
		Servo->RState |= SM_COMMUTATION;
		phase = 0;
	}
}

void TrapezoidalMotion::Calculate(float p0, float v0, float p1, float v1, float vel, float acc) {
    P0 = p0; V0 = v0; P1 = p1; V1 = v1; 
	D = P1 - P0;
	float S0 = v0 * fabsf(v0) / acc * 0.5F;
	float VT = (S0 > D)? -vel : vel;
	float A1 = (v0 > VT)? -acc : acc;
	float A3 = (VT > 0.0F)? -acc : acc;
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
	CurSegment = &Seg[0];
	phase = 1;
	time = 0;
	Servo->RPos = GPos = Seg[0].Pos; 
	Servo->RVel = GVel = Seg[0].Vel; 
	Servo->RAcc = GAcc = Seg[0].Acc; 
	Servo->RJerk = GJerk = 0;
	Servo->StartMotion();
}
void TrapezoidalMotion::Tick() {
	if (Servo->TPosChanged()) {
		if (Servo->GroupValidatePositionLoop()) {
			if (Servo->Gnax == 1) {
				Calculate(Servo->RPos, Servo->RVel, Servo->TPos, 0, Servo->Vel, Servo->Acc);
			} else {

			}
		}
	}
	if (phase != 0) {
		time += SECONDS_IN_TICK;
		while (1) {
			float T = time - CurSegment->StartTime;
			if (T < CurSegment->Duration) {
				float V = CurSegment->Acc*T;
				Servo->RVel = GVel = CurSegment->Vel + V;
				Servo->RPos = GPos = CurSegment->Pos + (CurSegment->Vel + V/2) * T;
				break;
			} else {
				if (phase <= 2) {
					phase++;
					CurSegment++;
					Servo->RAcc = GAcc = CurSegment->Acc;
				} else {	// Motion finished
					Servo->RPos = GPos = P1;
					Servo->RVel = Servo->RAcc = GVel = GAcc = 0;
					phase = 0;
					Servo->EndMotion();
					break;
				}
			}
		}
	} else {
	}
}
void TrapezoidalMotion::Kill() {
	// Seg[2].Acc = (GVel >= 0)? -KDec : KDec;
	// Seg[2].Duration = -GVel / Seg[2].Acc;
	// Seg[0].StartTime = Seg[0].Duration = Seg[1].StartTime = Seg[1].Duration = Seg[2].StartTime = 0;
	// Seg[2].Pos = GPos;
	// Seg[2].Vel = GVel;
	// P1 = Seg[2].Pos + Seg[2].Vel * Seg[2].Duration / 2;
	// GAcc = Seg[2].Acc;
	// CurSegment = &Seg[2];
	// time = 0;
	// phase = 3;
}

void GroupTrapezoidalMotion::Calculate(float s, float vel, float acc) {
	float v = vel;
	float t1 = v/acc;
	float s1 = v * 0.5F * t1;
	float s2 = s - 2*s1;
	float t2 = (s2 > 0) ? s2/v : 0;
	if (t2 == 0) {
		v = sqrtf(s * acc);
		t1 = v/acc;
		s1 = s * 0.5F;
	}
	Seg[0].StartTime = 0.0F;
	Seg[0].Duration = Seg[1].StartTime = t1;
	Seg[1].Duration = t2; Seg[2].StartTime = t1+t2;
	Seg[2].Duration = t1;
	Seg[0].Acc = acc; Seg[1].Acc = 0.0F; Seg[2].Acc = -acc;
	Seg[0].Vel = 0; Seg[1].Vel = Seg[2].Vel = v;
	Seg[0].Pos = 0; Seg[1].Pos = s1; Seg[2].Pos = s1 + s2;
	CurSegment = &Seg[0];
	phase = 1;
	time = 0;
	GPos = Seg[0].Pos; 
	GVel = Seg[0].Vel; 
	GAcc = Seg[0].Acc; 
	GJerk = 0;
	Servo->StartMotion();
}
void GroupTrapezoidalMotion::Tick() {
	if (Servo->GroupTPosChanged()) {
		if (Servo->GroupValidatePositionLoop()) {
			Servo->GroupGetRPos(p0); Servo->GroupGetTPos(p1);
			s = Euclid(p0, p1, Servo->Gnax);
			float _s = 1.0F / s;
			for (int i = 0; i < Servo->Gnax; i++) c[i] = (p1[i] - p0[i]) * _s;
			Calculate(s, MVel, MAcc);
			Servo->GroupSetRefs(p0, c);
		}
	}
	if (phase != 0) {
		time += SECONDS_IN_TICK;
		while (1) {
			float t = time - CurSegment->StartTime;
			if (t < CurSegment->Duration) {
				float V = CurSegment->Acc*t;
				GVel = CurSegment->Vel + V;
				GPos = CurSegment->Pos + (CurSegment->Vel + V/2) * t;
				Servo->GroupSetRefs(p0, c);
				break;
			} else {
				if (phase<=2) {
					phase++;
					CurSegment++;
					GAcc = CurSegment->Acc;
				} else {	// Motion finished
					GPos = s;
					GVel = GAcc = 0;
					phase = 0;
					Servo->GroupSetRefs(p0, c);
					Servo->EndMotion();
					break;
				}
			}
		}
	} else {
	}
}
void GroupTrapezoidalMotion::Kill() {
	// Seg[2].Acc = (GVel >= 0)? -KDec : KDec;
	// Seg[2].Duration = -GVel / Seg[2].Acc;
	// Seg[0].StartTime = Seg[0].Duration = Seg[1].StartTime = Seg[1].Duration = Seg[2].StartTime = 0;
	// Seg[2].Pos = GPos;
	// Seg[2].Vel = GVel;
	// P1 = Seg[2].Pos + Seg[2].Vel * Seg[2].Duration / 2;
	// GAcc = Seg[2].Acc;
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
	GJerk = Seg[0].Jerk;
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
				GAcc = CurSegment->Acc + dA;
				GVel = CurSegment->Vel + dV + dV1;
				GPos = CurSegment->Pos + (CurSegment->Vel + dV * 0.5F + dV1 * 0.333333333F) * T;
				break;
			} else {
				if (phase < 7) {
					phase++;
					CurSegment++;
					GJerk = CurSegment->Jerk;
				} else {	// Motion finished
					GPos = P1;
					GVel = GAcc = GJerk = 0;
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
		GPos = factor * point[0];
		GVel = factor * point[1];
		GAcc = factor * point[2];
		point += 3;
		current++;
	} else {
		GPos = P1;
		GVel = GAcc = 0;
		new(this) MotionBase();
	}
}

MinimumEnergyMotion::MinimumEnergyMotion(float p1, float v1) : TrapezoidalMotion((uint16_t)M_MINIMUMENERGY) {
	P0 = GPos; V0 = GVel; P1 = p1; V1 = v1;
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
				GPos = P1;
				GVel = GAcc = 0;
				new(this) MotionBase();
				return;
			}
		}
		if (phase == 1) {
			float acc = (CurSegment->Acc < 0)? MAcc : -MAcc;
			GAcc = acc + CurSegment->Acc * T;
			GVel = (acc + CurSegment->Acc * T * 0.5F) * T;
			GPos = CurSegment->Pos + (acc * 0.5F + CurSegment->Acc * T * 0.166666666F) * T * T;
		} else if (phase == 2) {
			GAcc = 0;
			GVel = CurSegment->Vel;
			GPos = CurSegment->Pos + CurSegment->Vel * T;
		} else {
			GAcc = CurSegment->Acc * T;
			GVel = CurSegment->Vel + CurSegment->Acc * T * T * 0.5F;
			GPos = CurSegment->Pos + (CurSegment->Vel + CurSegment->Acc * T * T * 0.166666666F) * T;
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
	s0 = Motions[0].GPos; s1 = Motions[1].GPos; f0 = t0; f1 = t1;
	float d0 = f0 - s0, d1 = f1 - s1;
	L = sqrtf(d0 * d0 + d1 * d1); k0 = d0 / L; k1 = d1 / L;
	new(&m) TrapezoidalMotion(L);
	new(&m) TrapezoidalMotion(L, 0);
	Motions[0].phase = m.phase; Motions[1].phase = 21;	// dependent
}

void Line2Motion::Tick() {
	m.Tick();
	if (m.phase > 0) {
		Motions[0].GPos = s0 + m.GPos * k0; Motions[1].GPos = s1 + m.GPos * k1;
		Motions[0].GVel = m.GVel * k0; Motions[1].GVel = m.GVel * k1;
		Motions[0].GAcc = m.GAcc * k0; Motions[1].GAcc = m.GAcc * k1;
		Motions[0].time = m.time; Motions[0].phase = m.phase;
		} else {
		Motions[0].GPos = f0; Motions[1].GPos = f1;
		Motions[0].GVel = 0; Motions[1].GVel = 0;
		Motions[0].GAcc = 0; Motions[1].GAcc = 0;
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
	s0 = Motions[0].GPos; s1 = Motions[1].GPos; f0 = t0; f1 = t1; c0 = ce0; c1 = ce1;
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
		float a = sa + m.GPos * ka, r = sr + m.GPos * kr;
		float s = sinf(a), c = cosf(a), ss = s, sc = c;
		if (dir < 0) {ss = -ss; sc = -sc;} 
		Motions[0].GPos = c0 + r * c; Motions[1].GPos = c1 + r * s;
		Motions[0].GVel = m.GVel * ss; Motions[1].GVel = m.GVel * sc;
		Motions[0].GAcc = - m.GVel * m.GVel * c + m.GAcc * ss; Motions[1].GAcc = - m.GVel * m.GVel * s + m.GAcc * sc;
		Motions[0].time = m.time; Motions[0].phase = m.phase;
	} else {
		Motions[0].GPos = f0; Motions[1].GPos = f1;
		Motions[0].GVel = 0; Motions[1].GVel = 0;
		Motions[0].GAcc = 0; Motions[1].GAcc = 0;
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

void TimeBased::Tick() {
	if (Servo->GroupTPosChanged()) {
		int n = (npoint < TB_FIFO) ? npoint : TB_FIFO - 1;
		Servo->GroupGetTPos(pt[n]);
		float tj = (MTj >= SECONDS_IN_TICK*0.5F) ? MTj : SECONDS_IN_TICK*0.5F;
		float ta = (MTa >= tj) ? MTa : tj;
		pt[n][NAX] = (MTv >= ta) ? MTv : ta;
		pt[n][NAX+1] = ta;
		pt[n][NAX+2] = tj;
	}
	if (phase == 0) {
		if ((npoint > 0) && Servo->GroupValidatePositionLoop()) {
			Servo->GroupGetRPos(p); 
			for (int i = 0; i < Servo->Gnax; i++) { 
				vm[i] = (pt[0][i] - p[i]) / pt[0][NAX];
				am[i] = vm[i] / pt[0][NAX+1];
				v[i] = a[i] = 0; 
				j[i] = am[0] / pt[0][NAX+2]; 
			}
			t = pt[0][NAX+2];
			time = 0;
			phase = 1;
		}
	} 
	if (phase != 0) {
		time += SECONDS_IN_TICK;
		while (1) {
			if (time < t) {
				for (int i = 0; i < Servo->Gnax; i++) {
					ServoStruct& s = Servo->GroupGetServo(i);
					s.RJerk = j[i];
					s.RAcc = a[i] + j[i] * time;
					s.RVel = v[i] + (a[i] + j[i] * time * 0.5F) * time;
					s.RPos = p[i] + (v[i] + (a[i] * 0.5F + j[i] * time * 0.166666666667F) * time) * time;
				}
				break;
			} else {
				if (phase==7) {
					for (int i = 0; i < Servo->Gnax; i++) {
						ServoStruct& s = Servo->GroupGetServo(i);
						// s.RPos = ;
						GVel = GAcc = 0;
						phase = 0;
						// Servo->GroupSetRefs(p0, c);
						Servo->EndMotion();
					}
					break;
				} else {	// Motion finished
					phase++;
					// CurSegment++;
					// GAcc = CurSegment->Acc;
				}
			}
		}
	} else {
	}
}

void Multipoint::Tick() {
	if (phase == 0) {
		while (Servo->FifoRead(p1)) {
			Servo->GroupGetRPos(p0);
			if (Changed(p0, p1, Servo->Gnax) && Servo->GroupValidatePositionLoop()) {
				Servo->GroupSetTPos(p1);
				p = EuclidAndCos(p0, p1, c, Servo->Gbax);
				v = Servo->Vel; a = Servo->Acc; j = Servo->Jerk;
				tj = a / j; 
				if (tj < SECONDS_IN_TICK*0.5F) { tj < SECONDS_IN_TICK*0.5F; j = a / tj; }
				ta = v / a;
				if (ta < tj) { ta = tj; a = v / ta; j = a / tj; }
				tv = p / v;
				if (tv < ta + tj) { tv = ta + tj; v = p / tv; a = v / ta; j = a / tj; }
				for (int i = 0; i < Servo->Gnax; i++) {
					j0[i] = j * c[i]; a0[i] = 0; v0[i] = 0;
				}
				time = t0 = 0;
				phase = 1;
				break;
			}
		}
	}
	if (phase != 0) {
		time += SECONDS_IN_TICK;
		float ti = time - t0, t = 0;
		while (t != ti) {
			if (phase == 1) {
				if (ti < tj) {
					t = ti; 
					for (int i = 0; i < Servo->Gnax; i++) {
						ServoStruct& s = Servo->GroupGetServo(i);
						s.RJerk = j0[i];
						s.RAcc = a0[i] + j0[i] * t;
						s.RVel = v0[i] + (a0[i] + j0[i] * t * 0.5F) * t;
						s.RPos = p0[i] + (v0[i] + (a0[i] * 0.5F + j0[i] * t * 0.166666666667F) * t) * t;
					}
				} else { 
					phase = 2; 
					t = tj; t0 += t; ti -= t;
					for (int i = 0; i < Servo->Gnax; i++) {
						p0[i] = p0[i] + (v0[i] + (a0[i] * 0.5F + j0[i] * t * 0.166666666667F) * t) * t;
						v0[i] = v0[i] + (a0[i] + j0[i] * t * 0.5F) * t;
						a0[i] = a0[i] + j0[i] * t;
					}
				}
			} else if (phase == 2) {
				if (ti < ta - tj) {
					t = ti; 
					for (int i = 0; i < Servo->Gnax; i++) {
						ServoStruct& s = Servo->GroupGetServo(i);
						s.RJerk = 0;
						s.RAcc = a0[i];
						s.RVel = v0[i] + a0[i] * t;
						s.RPos = p0[i] + (v0[i] + a0[i] * 0.5F * t) * t;
					}
				} else { 
					phase = 3; 
					t = ta - tj; 
					if (t > 0) {
						t0 += t; ti -= t; 
						for (int i = 0; i < Servo->Gnax; i++) {
							p0[i] = p0[i] + (v0[i] + a0[i] * 0.5F * t) * t;
							v0[i] = v0[i] + a0[i] * t;
						}
					}
				}
			} else if (phase == 3) {
				if (ti < tj) {
					t = ti; 
					for (int i = 0; i < Servo->Gnax; i++) {
						ServoStruct& s = Servo->GroupGetServo(i);
						s.RJerk = -j0[i];
						s.RAcc = a0[i] - j0[i] * t;
						s.RVel = v0[i] + (a0[i] - j0[i] * t * 0.5F) * t;
						s.RPos = p0[i] + (v0[i] + (a0[i] * 0.5F - j0[i] * t * 0.166666666667F) * t) * t;
					}
				} else { 
					phase = 4; 
					t = tj; t0 += t; ti -= t;
					for (int i = 0; i < Servo->Gnax; i++) {
						p0[i] = p0[i] + (v0[i] + (a0[i] * 0.5F - j0[i] * t * 0.166666666667F) * t) * t;
						v0[i] = v0[i] + (a0[i] - j0[i] * t * 0.5F) * t;
						a0[i] = a0[i] - j0[i] * t;
					}
				}
			} else if (phase == 4) {
				if (ti < tv - ta - tj) {
					t = ti; 
					for (int i = 0; i < Servo->Gnax; i++) {
						ServoStruct& s = Servo->GroupGetServo(i);
						s.RJerk = 0;
						s.RAcc = 0;
						s.RVel = v0[i];
						s.RPos = p0[i] + v0[i] * t;
					}
				} else { 
					phase = 5;
					t = tv - ta - tj; 
					if (t > 0) {
						t0 += t; ti -= t;
						for (int i = 0; i < Servo->Gnax; i++) {
							p0[i] = p0[i] + v0[i] * t;
						}
					}
					while (Servo->FifoRead(p2)) {
						if (Changed(p1, p2, Servo->Gnax)) {
							Servo->GroupSetTPos(p2);
							p = EuclidAndCos(p1, p2, c, Servo->Gbax);	// blending
							v = Servo->Vel; 
							tv = p / v;
							if (tv < ta + tj) { tv = ta + tj; v = p / tv; }
							a = v / ta; j = a / tj; 
							for (int i = 0; i < Servo->Gnax; i++) {
								float v1 = v * c[i];
								float a1 = (v1 - v0[i]) / ta;
								j0[i] = a1 / tj;
								p1[i] = p2[i];
							}
							phase = 1;
							break;
						}
					}
					if (phase == 5) {
						for (int i = 0; i < Servo->Gnax; i++) {
							j0[i] = j * c[i]; a0[i] = 0;
						}
					}
				}
			} else if (phase == 5) {
				if (ti < tj) {
					t = ti; 
					for (int i = 0; i < Servo->Gnax; i++) {
						ServoStruct& s = Servo->GroupGetServo(i);
						s.RJerk = -j0[i];
						s.RAcc = a0[i] - j0[i] * t;
						s.RVel = v0[i] + (a0[i] - j0[i] * t * 0.5F) * t;
						s.RPos = p0[i] + (v0[i] + (a0[i] * 0.5F - j0[i] * t * 0.166666666667F) * t) * t;
					}
				} else { 
					phase = 6; 
					t = tj; t0 += t; ti -= t;
					for (int i = 0; i < Servo->Gnax; i++) {
						p0[i] = p0[i] + (v0[i] + (a0[i] * 0.5F - j0[i] * t * 0.166666666667F) * t) * t;
						v0[i] = v0[i] + (a0[i] - j0[i] * t * 0.5F) * t;
						a0[i] = a0[i] - j0[i] * t;
					}
				}
			} else if (phase == 6) {
				if (ti < ta - tj) {
					t = ti; 
					for (int i = 0; i < Servo->Gnax; i++) {
						ServoStruct& s = Servo->GroupGetServo(i);
						s.RJerk = 0;
						s.RAcc = a0[i];
						s.RVel = v0[i] + a0[i] * t;
						s.RPos = p0[i] + (v0[i] + a0[i] * 0.5F * t) * t;
					}
				} else { 
					phase = 7; 
					t = ta - tj; 
					if (t > 0) {
						t0 += t; ti -= t; 
						for (int i = 0; i < Servo->Gnax; i++) {
							p0[i] = p0[i] + (v0[i] + a0[i] * 0.5F * t) * t;
							v0[i] = v0[i] + a0[i] * t;
						}
					}
				}
			} else if (phase == 7) {
				if (ti < tj) {
					t = ti; 
					for (int i = 0; i < Servo->Gnax; i++) {
						ServoStruct& s = Servo->GroupGetServo(i);
						s.RJerk = j0[i];
						s.RAcc = a0[i] + j0[i] * t;
						s.RVel = v0[i] + (a0[i] + j0[i] * t * 0.5F) * t;
						s.RPos = p0[i] + (v0[i] + (a0[i] * 0.5F + j0[i] * t * 0.166666666667F) * t) * t;
					}
				} else { 
					phase = 0; 
					t = ti;
					for (int i = 0; i < Servo->Gnax; i++) {
						ServoStruct& s = Servo->GroupGetServo(i);
						s.RJerk = 0;
						s.RAcc = 0;
						s.RVel = 0;
						s.RPos = p1[i];
					}
				}
			} else if (phase == 100) {		// Kill
				for (int i = 0; i < Servo->Gnax; i++) {
					ServoStruct& s = Servo->GroupGetServo(i);
					if (s.RVel = 0) {
						s.Jerk = s.Acc = 0;
					} else {
						t = SECONDS_IN_TICK;
						float dv = s.Acc * t;
						if (fabsf(s.RVel) > dv) {
							if (s.RVel > 0) dv = -dv;
							s.RPos += (s.RVel + 0.5F * dv) * t; 
							s.RVel += dv;
							s.RAcc = (dv > 0) ? s.Acc : -s.Acc;
							s.RJerk = 0;
						} else {
							s.FifoFlush();
							phase = 0;
							t = fabsf(s.RVel) / s.Acc;
							dv = - s.RVel;
							s.RPos -= 0.5F * dv * t;
							s.RVel = s.RAcc = s.RJerk = 0;
						}
					}
				}
			}
		}
	}
}
void Multipoint::Kill() {
	Servo->FifoFlush(); 
	phase = 100; 
}

void MotionInit() {
	FillCubicRootTable();
	for (int i = 0; i < NAX; i++) { 
		Motions[i].Init(i); 
	}
}
