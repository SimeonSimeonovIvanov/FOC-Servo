#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "foc.h"

int run_foc()
{
	MC_FOC stFoc;

	mcFocInitStruct(&stFoc);
	mcFoc( &stFoc );

	return 0;
}

void mcFoc(LP_MC_FOC lpFoc)
{
	FILE *fp;
	float angle;
	float Ia = 0, Ib = 0;
	
	#pragma warning(disable : 4996)
	if(!(fp = fopen("foc_test.csv", "w"))) {
		return;
	}

	for( angle = 0; angle <= 360; angle += 1.0f ) {
		Ia = 10.0f * sin( foc_deg_to_rad( angle +   0 ) );
		Ib = 10.0f * sin( foc_deg_to_rad( angle - 120 ) );

		mcFocSetAngle( lpFoc, angle );
		mcFocSetCurrent( lpFoc, Ia, Ib );

		mcClark( lpFoc );
		mcPark( lpFoc );

		lpFoc->Vd = 0; lpFoc->Id;
		lpFoc->Vq = 0; lpFoc->Iq;

		mcInvPark( lpFoc );
		mcInvClark( lpFoc );

		mcFocSVPWM( lpFoc );

		/*fprintf(fp, "Angle:\t%3.1f\n", lpFoc->angle);
		fprintf(fp, "Ia:\t%3.1f\tIb:\t%3.1f\n\n", lpFoc->Ia, lpFoc->Ib);
		fprintf(fp, "Ialpha:\t%3.1f\tIbeta:\t%3.1f\n", lpFoc->Ialpha, lpFoc->Ibeta);
		fprintf(fp, "Iq:\t%3.1f\tId:\t%3.1f\n", lpFoc->Iq, lpFoc->Id);
		fprintf(fp, "Valpha:\t%3.1f\tVbeta:\t%3.1f\n\n", lpFoc->Valpha, lpFoc->Vbeta);
		fprintf(fp, "Va:\t%3.1f\tVb:\t%3.1f\tVc:\t%3.1f\n", lpFoc->Va, lpFoc->Vb, lpFoc->Vc);
		fputs("--------------------------------------------------------------------\n\n",fp);*/
		//fprintf(fp, "%3.1f\t%3.1f\t%3.1f\n", lpFoc->Va, lpFoc->Vb, lpFoc->Vc);
		//fprintf(fp, "%3.1f\t%3.1f\n", lpFoc->Valpha, lpFoc->Vbeta);

		//fprintf(fp, "%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f\n", lpFoc->angle, lpFoc->Ia, lpFoc->Ib, lpFoc->Va, lpFoc->Vb, lpFoc->Vc);
		//fprintf(fp, "%E;%E;%E;%E;%E;%E;%E;%E\n", lpFoc->Ia, lpFoc->Ib, lpFoc->Ialpha, lpFoc->Ibeta, lpFoc->Valpha, lpFoc->Vbeta, lpFoc->Va, lpFoc->Vb, lpFoc->Vc);
	}

	fclose(fp);
}

#define ONE_BY_SQRT3	( SQRT3 )
#define TWO_BY_SQRT3	( 2.0f * SQRT3 )
#define SQRT3_BY_2		( 0.86602540378f )

typedef unsigned long  uint32_t;

// Magnitude must not be larger than sqrt(3)/2, or 0.866
static void svm
(
	float alpha, float beta, uint32_t PWMHalfPeriod,
	uint32_t* tAout, uint32_t* tBout, uint32_t* tCout
)
{
	uint32_t sector;

	if (beta >= 0.0f) {
		if (alpha >= 0.0f) {
			//quadrant I
			if (ONE_BY_SQRT3 * beta > alpha)
				sector = 2;
			else
				sector = 1;
		}
		else {
			//quadrant II
			if (-ONE_BY_SQRT3 * beta > alpha)
				sector = 3;
			else
				sector = 2;
		}
	}
	else {
		if (alpha >= 0.0f) {
			//quadrant IV5
			if (-ONE_BY_SQRT3 * beta > alpha)
				sector = 5;
			else
				sector = 6;
		}
		else {
			//quadrant III
			if (ONE_BY_SQRT3 * beta > alpha)
				sector = 4;
			else
				sector = 5;
		}
	}

	// PWM timings
	uint32_t tA, tB, tC;

	switch (sector) {

		// sector 1-2
	case 1: {
		// Vector on-times
		uint32_t t1 = (alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t2 = (TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tA = (PWMHalfPeriod - t1 - t2) / 2;
		tB = tA + t1;
		tC = tB + t2;

		break;
	}

			// sector 2-3
	case 2: {
		// Vector on-times
		uint32_t t2 = (alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t3 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tB = (PWMHalfPeriod - t2 - t3) / 2;
		tA = tB + t3;
		tC = tA + t2;

		break;
	}

			// sector 3-4
	case 3: {
		// Vector on-times
		uint32_t t3 = (TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t4 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tB = (PWMHalfPeriod - t3 - t4) / 2;
		tC = tB + t3;
		tA = tC + t4;

		break;
	}

			// sector 4-5
	case 4: {
		// Vector on-times
		uint32_t t4 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t5 = (-TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tC = (PWMHalfPeriod - t4 - t5) / 2;
		tB = tC + t5;
		tA = tB + t4;

		break;
	}

			// sector 5-6
	case 5: {
		// Vector on-times
		uint32_t t5 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t6 = (alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tC = (PWMHalfPeriod - t5 - t6) / 2;
		tA = tC + t5;
		tB = tA + t6;

		break;
	}

			// sector 6-1
	case 6: {
		// Vector on-times
		uint32_t t6 = (-TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t1 = (alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tA = (PWMHalfPeriod - t6 - t1) / 2;
		tC = tA + t1;
		tB = tC + t6;

		break;
	}
	}

	*tAout = tA;
	*tBout = tB;
	*tCout = tC;
}

/**
* Truncate the magnitude of a vector.
*
* @param x
* The first component.
*
* @param y
* The second component.
*
* @param max
* The maximum magnitude.
*
* @return
* True if saturation happened, false otherwise
*/
bool utils_saturate_vector_2d(float *x, float *y, float max) {
	bool retval = false;
	float mag = sqrtf(*x * *x + *y * *y);
	max = fabsf(max);

	if (mag < 1e-10) {
		mag = 1e-10;
	}

	if (mag > max) {
		const float f = max / mag;
		*x *= f;
		*y *= f;
		retval = true;
	}

	return retval;
}

void mcFocSVPWM5(LP_MC_FOC lpFoc)
{
	float mod_alpha, mod_beta;
	float mod_alpha_comp = 0;
	float mod_beta_comp = 0;
	float mod_d, mod_q;
	float max_duty;
	float Vd, Vq;
	float v_bus;
	
	max_duty = 8000;

	Vd = 0; lpFoc->Vd;
	Vq = 5; lpFoc->Vq;
	v_bus = 50;

	mod_d = Vd / ((2.0 / 3.0) * v_bus);
	mod_q = Vq / ((2.0 / 3.0) * v_bus);

	/*utils_saturate_vector_2d
	(
		(float*)&mod_d, (float*)&mod_q,
		SQRT3_BY_2 * max_duty
	);

	utils_saturate_vector_2d
	(
		(float*)&Vd, (float*)&Vq,
		(2.0 / 3.0) * max_duty * SQRT3_BY_2 * v_bus
	);*/

	mod_alpha = lpFoc->fCosAngle * mod_d - lpFoc->fSinAngle * mod_q;
	mod_beta  = lpFoc->fCosAngle * mod_q + lpFoc->fSinAngle * mod_d;

	// Set output (HW Dependent)
	uint32_t duty1, duty2, duty3, top = 10000;
	//top = TIM1->ARR;
	svm(-mod_alpha, -mod_beta, top, &duty1, &duty2, &duty3);
	//TIMER_UPDATE_DUTY(duty1, duty2, duty3);

	lpFoc->PWM1 = duty1 / 100;
	lpFoc->PWM2 = duty2 / 100;
	lpFoc->PWM3 = duty3 / 100;
}

void mcFocSVPWM4(LP_MC_FOC lpFoc)
{
	float Uref1, Uref2, Uref3;
	int A = 0, B = 0, C = 0;
  //int sectorIndex, arrSectorMap[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	int sectorIndex, arrSectorMap[8] = { 0, 2, 6, 1, 4, 3, 5, 7 };
	float X, Y, Z;
	float T0, Tpwm, T1, T2;
	float Udc, SQRT3_Tpwm, SQRT3_Tpwm_Udc;
	float Ta, Tb, Tc;
	float PWM1, PWM2, PWM3;

	//////////////////////////////////////////////////////////////////
	Uref1 = lpFoc->Vbeta;
	Uref2 = (+(SQRT3 * lpFoc->Valpha) - lpFoc->Vbeta);
	Uref3 = (-(SQRT3 * lpFoc->Valpha) - lpFoc->Vbeta);

	if (Uref1 > 0.0f) {
		A = 1;
	}

	if (Uref2 > 0.0f) {
		B = 1;
	}

	if (Uref3 > 0.0f) {
		C = 1;
	}

	sectorIndex = (C*4) + (B*2) + A;
	lpFoc->sector = arrSectorMap[ sectorIndex ];
	//////////////////////////////////////////////////////////////////
	Tpwm = 100;
	Udc = 0.65;

	SQRT3_Tpwm = SQRT3 * Tpwm;
	SQRT3_Tpwm_Udc = SQRT3_Tpwm / Udc;

	X = SQRT3_Tpwm_Udc * lpFoc->Vb;
	Y = SQRT3_Tpwm_Udc * (+(SQRT3_DIV2 * lpFoc->Valpha) + lpFoc->Vbeta);
	Z = SQRT3_Tpwm_Udc * (-(SQRT3_DIV2 * lpFoc->Valpha) + lpFoc->Vbeta);

	static int index = 0;

	switch (lpFoc->sector) {
	case 0:
	case 7:
	break;

	case 1:
		T1 = Z;
		T2 = Y;
	break;

	case 2:
		T1 = Y;
		T2 = -X;
	break;

	case 3:
		T1 = -Z;
		T2 = X;
	break;

	case 4:
		T1 = -X;
		T2 = Z;
	break;

	case 5:
		T1 = X;
		T2 = -Y;
	break;

	case 6:
		T1 = -Y;
		T2 = -Z;
	break;
	}
	T0 = Tpwm - T1 - T2;

	Ta = (Tpwm - T1 - T2) / 4;
	Tb = Ta + ( T1 * 0.5f );
	Tc = Tb + ( T2 * 0.5f );
	
	switch (lpFoc->sector) {
	case 0:
	case 7:
	break;

	case 1:
		PWM1 = Tb;
		PWM2 = Ta;
		PWM3 = Tc;
	break;

	case 2:
		PWM1 = Ta;
		PWM2 = Tc;
		PWM3 = Tb;
	break;

	case 3:
		PWM1 = Ta;
		PWM2 = Tb;
		PWM3 = Tc;
	break;

	case 4:
		PWM1 = Tc;
		PWM2 = Tb;
		PWM3 = Ta;
	break;

	case 5:
		PWM1 = Tc;
		PWM2 = Ta;
		PWM3 = Tb;
	break;

	case 6:
		PWM1 = Tb;
		PWM2 = Tc;
		PWM3 = Ta;
	break;
	}

	lpFoc->PWM1 = PWM1/100;
	lpFoc->PWM2 = PWM2/100;
	lpFoc->PWM3 = PWM3/100;
}

void mcFocSVPWM3(LP_MC_FOC lpFoc)
{
	int   sector;
	float X, Y, Z;
	int   t1, t2;
	int   T1, T2, T3;
	int pwma, pwmb, pwmc;

	//Modified Inverse Clarke
	X = lpFoc->Vbeta;
	Y = 0.5 * ( lpFoc->Vbeta + (1.73205 * lpFoc->Valpha) );
	Z = 0.5 * ( lpFoc->Vbeta - (1.73205 * lpFoc->Valpha) );

	//Get Sector
	//sector = ip->theta / 170; //60 degrees expressed in counts
	// Apply Math per PWM Sector - Sector 0

	sector = lpFoc->angle / 60;

	switch (sector) {
	case 6:  //Wrap
	case 0:  //
		t1 = (int)X;
		t2 = (int)-Z;
		break;

	case 1:  //
		t1 = (int)Y;
		t2 = (int)Z;
		break;

	case 2:  //
		t1 = (int)-Y;
		t2 = (int)X;
		break;

	case 3:  //
		t1 = (int)Z;
		t2 = (int)-X;
		break;

	case 4:  //
		t1 = (int)-Z;
		t2 = (int)-Y;
		break;

	case 5:  //
		t1 = (int)-X;
		t2 = (int)Y;
		break;
	}

	//Now Calculate Periods
	T1 = (100 - t1 - t2) / 2;   //Max is Value is 1024, Max Input is 512
	T2 = T1 + t1;
	T3 = T2 + t2;

	//Now setup outputs
	//Apply Math per PWM Sector - Sector 0
	switch (sector)
	{
	case 6:
	case 0:  //
		pwma = T3;
		pwmb = T2;
		pwmc = T1;
		break;

	case 1:  //
		pwma = T2;
		pwmb = T3;
		pwmc = T1;
		break;

	case 2:  //
		pwma = T1;
		pwmb = T3;
		pwmc = T2;
		break;

	case 3:  //
		pwma = T1;
		pwmb = T2;
		pwmc = T3;
		break;

	case 4:  //
		pwma = T2;
		pwmb = T1;
		pwmc = T3;
		break;

	case 5:  //
		pwma = T3;
		pwmb = T1;
		pwmc = T2;
		break;
	}

	//PWM Output Now
	lpFoc->PWM1 = pwma / 1;
	lpFoc->PWM2 = pwmb / 1;
	lpFoc->PWM3 = pwmc / 1;

	lpFoc->sector = sector;
	static int index = 0, sector_old = 0;
	if (sector_old != lpFoc->sector) {
		lpFoc->arrSector[index++] = lpFoc->sector;
		sector_old = lpFoc->sector;
	}
}

void mcFocSVPWM0(LP_MC_FOC lpFoc)
{
	/*float valpha, vbeta;
	float vb, va_int, vb_int, vc_int;

	valpha = lpFoc->Valpha;
	vbeta = lpFoc->Vbeta;

	// Va = Valpha
	// Vb = -1/2 * Valpha + V3/2 * Vbeta
	// Vc = -1/2 * Valpha - V3/2 * Vbeta
	va_int = valpha;
	vb = SQRT3_DIV2 * vbeta;
	vb_int = -( va_int * 0.5f ) + vb;
	vc_int = -( va_int * 0.5f ) - vb;

	lpFoc->PWM1 = va_int*50;
	lpFoc->PWM2 = vb_int*50;
	lpFoc->PWM3 = vc_int*50;*/

	//return;
	mcInvClark(lpFoc);

	lpFoc->PWM1 = (0.5 + lpFoc->Va) * 50;
	lpFoc->PWM3 = (0.5 + lpFoc->Vb) * 50;
	lpFoc->PWM2 = (0.5 + lpFoc->Vc) * 50;
}

void mcFocSVPWM2(LP_MC_FOC lpFoc)
{
	int hTimePhA, hTimePhB, hTimePhC;
	int Ualpha, Ubeta;
	int T, T_2, T_4;
	int X, Y, Z;
	int sector;

	T = 100;
	T_2 = T * 0.50f;
	T_4 = T * 0.25f;

	Ualpha = T * SQRT3 * lpFoc->Valpha;
	Ubeta = -T * lpFoc->Vbeta;

	X = Ubeta;
	Y = (Ualpha + Ubeta) * 0.5f;
	Z = (Ubeta - Ualpha) * 0.5f;

	if ( Y < 0 ) {
		if (Z < 0) {
			sector = 5;
		} else {
			if (X > 0) {
				sector = 3;
			} else {
				sector = 4;
			}
		}
	} else {
		if (Z < 0) {
			if (X > 0) {
				sector = 1;
			} else {
				sector = 6;
			}
		} else {
			sector = 2;
		}
	}

	switch (sector) {
	case 1:
	case 4:
		hTimePhA = T_4 + ( (T_2 + X - Z) * 0.5f );
		hTimePhB = hTimePhA + Z;
		hTimePhC = hTimePhB - X;
	 break;

	case 2:
	case 5:
		hTimePhA = T_4 + ( (T_2 + Y - Z) * 0.5f );
		hTimePhB = hTimePhA + Z;
		hTimePhC = hTimePhA - Y;
	 break;

	case 3:
	case 6:
		hTimePhA = T_4 + ( (T_2 + Y - X) * 0.5f );
		hTimePhC = hTimePhA - Y;
		hTimePhB = hTimePhC + X;
	 break;
	}

	lpFoc->PWM1 = hTimePhA;
	lpFoc->PWM2 = hTimePhB;
	lpFoc->PWM3 = hTimePhC;

	lpFoc->sector = sector;
	static int index = 0, sector_old = 0;
	if (sector_old != lpFoc->sector) {
		lpFoc->arrSector[index++] = lpFoc->sector;
		sector_old = lpFoc->sector;
	}
}

#define PWM_PERIOD	25
#define T			( PWM_PERIOD * 4 )
#define T_SQRT3     ( T * SQRT3 )
#define div			( 1 )

void mcFocSVPWM1(LP_MC_FOC lpFoc) // +
{
	int bSector;
	float wX, wY, wZ;
	float wUAlpha, wUBeta;
	float hTimePhA, hTimePhB, hTimePhC;

	wUAlpha = +(lpFoc->Valpha * T_SQRT3);
	wUBeta  = -(lpFoc->Vbeta  * T);

	wX = wUBeta;
	wY = (wUBeta + wUAlpha) * 0.5f;
	wZ = (wUBeta - wUAlpha) * 0.5f;

	if(wY<0) {
		if(wZ<0) {
			bSector = 5;
		} else {// wZ >= 0
			if (wX <= 0) {
				bSector = 4;
			} else {// wX > 0
				bSector = 3;
			}
		}
	} else { // wY > 0
		if(wZ >= 0) {
			bSector = 2;
		} else { // wZ < 0
			if(wX <= 0) {
				bSector = 6;
			} else { // wX > 0
				bSector = 1;
			}
		}
	}

	switch (bSector) {
	case 1:
	case 4:
		hTimePhA = (T / 8) + ((((T + wX) - wZ) / 2) / div);
		hTimePhB = hTimePhA + wZ / div;
		hTimePhC = hTimePhB - wX / div;
	 break;

	case 2:
	case 5:
		hTimePhA = (T / 8) + ((((T + wY) - wZ) / 2) / div);
		hTimePhB = hTimePhA + wZ / div;
		hTimePhC = hTimePhA - wY / div;
	 break;

	case 3:
	case 6:
		hTimePhA = (T / 8) + ((((T - wX) + wY) / 2) / div);
		hTimePhC = hTimePhA - wY / div;
		hTimePhB = hTimePhC + wX / div;
	 break;

	default:
		break;
	}
	
	lpFoc->PWM1 = hTimePhA;
	lpFoc->PWM2 = hTimePhB;
	lpFoc->PWM3 = hTimePhC;

	lpFoc->sector = bSector;
	static int index = 0, sector_old = 0;
	if (sector_old != lpFoc->sector) {
		lpFoc->arrSector[index++] = lpFoc->sector;
		sector_old = lpFoc->sector;
	}
}

void mcFocSVPWM(LP_MC_FOC lpFoc)
{	
	float X, Y, Z;
	float taon, tbon, tcon;
	float PWMPRD = 20, t1, t2;
	volatile int PWM1, PWM2, PWM3;
	volatile int A = 0, B = 0 , C = 0;

	if ( lpFoc->Va > 0 ) {
		A = 1;
	}
	if ( lpFoc->Vb > 0 ) {
		B = 1;
	}
	if ( lpFoc->Vc > 0 ) {
		C = 1;
	}

	lpFoc->sector = (C << 2) + (B << 1) + A;

	float Vdc = ( 380.0f / 380.0f );
	float Vdcinvt = PWMPRD / Vdc;

	X = (      SQRT3 * Vdcinvt * lpFoc->Vbeta );
	Y = ( SQRT3_DIV2 * Vdcinvt * lpFoc->Vbeta ) + ( .5f * Vdcinvt * lpFoc->Valpha );
	Z = ( SQRT3_DIV2 * Vdcinvt * lpFoc->Vbeta ) - ( .5f * Vdcinvt * lpFoc->Valpha);

	switch( lpFoc->sector ) {
	case 0:
	 break;

	case 1:
		t1 = Z;
		t2 = Y;
	 break;

	case 2:
		t1 = Y;
		t2 = -X;
	 break;

	case 3:
		t1 = -Z;
		t2 = X;
	 break;

	case 4:
		t1 = -X;
		t2 = Z;
	 break;

	case 5:
		t1 = X;
		t2 = -Y;
	 break;

	case 6:
		t1 = -Y;
		t2 = -Z;
	 break;

	case 7:
	 break;
	}

	if ((t1 + t2) > PWMPRD) {
		t1 = t1 * (PWMPRD / (t1 + t2));
		t2 = t2 * (PWMPRD / (t1 + t2));
	}

	taon = (PWMPRD - t1 - t2 ) * 0.5f;
	tbon = taon + t1;
	tcon = tbon + t2;

	switch (lpFoc->sector) {
	case 0:
	 break;

	case 1:
		PWM1 = tbon;
		PWM2 = taon;
		PWM3 = tcon;
	 break;

	case 2:
		PWM1 = tbon;
		PWM2 = tcon;
		PWM3 = tbon;
	 break;

	case 3:
		PWM1 = taon;
		PWM2 = tbon;
		PWM3 = tcon;
	 break;

	case 4:
		PWM1 = tcon;
		PWM2 = tbon;
		PWM3 = taon;
	 break;

	case 5:
		PWM1 = tcon;
		PWM2 = taon;
		PWM3 = tbon;
	 break;

	case 6:
		PWM1 = tbon;
		PWM2 = tcon;
		PWM3 = taon;
	 break;

	case 7:
	 break;
	}

	lpFoc->PWM1 = PWM1;
	lpFoc->PWM2 = PWM2;
	lpFoc->PWM3 = PWM3;

	static int index = 0, sector_old = 0;
	if (sector_old != lpFoc->sector) {
		lpFoc->arrSector[index++] = lpFoc->sector;
		sector_old = lpFoc->sector;
	}
}

float fSinAngle(int angle)
{
	return sinf(foc_deg_to_rad(angle));
}

float fCosAngle(int angle)
{
	return cosf(foc_deg_to_rad(angle));
}

void mcFocSetAngle(LP_MC_FOC lpFoc, int angle)
{
	lpFoc->angle = (float)angle;
	lpFoc->fSinAngle = fSinAngle(angle);
	lpFoc->fCosAngle = fCosAngle(angle);
}

void mcFocSetCurrent(LP_MC_FOC lpFoc, float Ia, float Ib)
{
	lpFoc->Ia = Ia;
	lpFoc->Ib = Ib;
}

void mcFocInitStruct(LP_MC_FOC lpFoc)
{
	memset( lpFoc, 0, sizeof(MC_FOC) );
}

void mcClark(LP_MC_FOC lpFoc)
{	
	lpFoc->Ialpha = lpFoc->Ia;
	lpFoc->Ibeta = (lpFoc->Ia + 2 * lpFoc->Ib) * divSQRT3;
}

void mcPark(LP_MC_FOC lpFoc)
{
	lpFoc->Id = +lpFoc->Ialpha * lpFoc->fCosAngle + lpFoc->Ibeta  * lpFoc->fSinAngle;
	lpFoc->Iq = -lpFoc->Ialpha * lpFoc->fSinAngle + lpFoc->Ibeta  * lpFoc->fCosAngle;
}

void mcInvPark(LP_MC_FOC lpFoc)
{
	lpFoc->Valpha = lpFoc->Vd * lpFoc->fCosAngle - lpFoc->Vq * lpFoc->fSinAngle;
	lpFoc->Vbeta  = lpFoc->Vd * lpFoc->fSinAngle + lpFoc->Vq * lpFoc->fCosAngle;
}

void mcInvClark(LP_MC_FOC lpFoc)
{
	lpFoc->Vb = lpFoc->Vbeta;
	lpFoc->Va = (-lpFoc->Vbeta + (SQRT3 * lpFoc->Valpha)) * 0.5f;
	lpFoc->Vc = (-lpFoc->Vbeta - (SQRT3 * lpFoc->Valpha)) * 0.5f;

	/*lpFoc->Va = lpFoc->Valpha;
	lpFoc->Vb = ( -lpFoc->Valpha + ( SQRT3 * lpFoc->Vbeta ) ) * 0.5f;
	lpFoc->Vc = ( -lpFoc->Valpha - ( SQRT3 * lpFoc->Vbeta ) ) * 0.5f;*/
}

void mcFocSVPWM_TI(LP_MC_FOC lpFoc)
{
	int sector, A = 0, B = 0, C = 0;
	float X, Y, Z;
	float t1, t2;
	float taon, tbon, tcon;
	float dcinvt = 50;

	if (lpFoc->Va >= 0.0f) A = 1;
	if (lpFoc->Vb >= 0.0f) B = 1;
	if (lpFoc->Vc >= 0.0f) C = 1;

	sector = A + 2 * B + 4 * C;

	X = (SQRT3 * lpFoc->Vbeta)*dcinvt;
	Y = (SQRT3_DIV2 * lpFoc->Vbeta*dcinvt) + ((3 / 2)*lpFoc->Valpha*dcinvt);
	Z = (SQRT3_DIV2 * lpFoc->Vbeta*dcinvt) - ((3 / 2)*lpFoc->Valpha*dcinvt);

	switch (sector) {
	case 1:
		t1 = Z; t2 = Y;
		break;

	case 2:
		t1 = Y; t2 = -X;
		break;

	case 3:
		t1 = -Z; t2 = X;
		break;

	case 4:
		t1 = -X; t2 = Z;
		break;

	case 5:
		t1 = X; t2 = -Y;
		break;

	case 6:
		t1 = -Y; t2 = -Z;
		break;
	}

	taon = (50 - t1 - t2) / 2;
	tbon = taon + t1;
	tcon = tbon + t2;

	switch (sector) {
	case 1:
		lpFoc->PWM1 = tbon;
		lpFoc->PWM2 = taon;
		lpFoc->PWM3 = tcon;
	break;

	case 2:
		lpFoc->PWM1 = taon;
		lpFoc->PWM2 = tcon;
		lpFoc->PWM3 = tbon;
	break;

	case 3:
		lpFoc->PWM1 = taon;
		lpFoc->PWM2 = tbon;
		lpFoc->PWM3 = tcon;
	break;

	case 4:
		lpFoc->PWM1 = tcon;
		lpFoc->PWM2 = tbon;
		lpFoc->PWM3 = taon;
	break;

	case 5:
		lpFoc->PWM1 = tcon;
		lpFoc->PWM2 = taon;
		lpFoc->PWM3 = tbon;
	break;

	case 6:
		lpFoc->PWM1 = tbon;
		lpFoc->PWM2 = tcon;
		lpFoc->PWM3 = taon;
	break;
	}

	static int index = 0, sector_old = 0;
	if (sector_old != lpFoc->sector) {
		lpFoc->arrSector[index++] = lpFoc->sector;
		sector_old = lpFoc->sector;
	}
}