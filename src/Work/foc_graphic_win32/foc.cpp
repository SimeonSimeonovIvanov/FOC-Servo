#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "foc.h"

float svpwm_sin3_table[4096];

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

		mcFocSPWM( lpFoc );

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
	if (1) {
		lpFoc->Vb = lpFoc->Vbeta;
		lpFoc->Va = (-lpFoc->Vbeta + (SQRT3 * lpFoc->Valpha)) *0.5f;
		lpFoc->Vc = (-lpFoc->Vbeta - (SQRT3 * lpFoc->Valpha)) *0.5f;
	} else {
		lpFoc->Va = lpFoc->Valpha;
		lpFoc->Vb = (-lpFoc->Valpha + (SQRT3 * lpFoc->Vbeta)) *0.5f;
		lpFoc->Vc = (-lpFoc->Valpha - (SQRT3 * lpFoc->Vbeta)) *0.5f;
	}
}

void mcUsrefLimit(LP_MC_FOC lpFoc)
{
	const float limit = 0.999f; SQRT3_DIV2;
	float Usref, scale, test;

	Usref = sqrtf(lpFoc->Vd * lpFoc->Vd + lpFoc->Vq * lpFoc->Vq);

	if (Usref > limit) {
		scale = limit / Usref;
		lpFoc->Vd *= scale;
		lpFoc->Vq *= scale;

		test = sqrtf(lpFoc->Vd * lpFoc->Vd + lpFoc->Vq * lpFoc->Vq);
		test *= 1.0f;
	}
}

float fLimitValue(float value, float limit)
{
	//float temp;

	if (value > limit) {
		//temp = limit / value;
		//value *= temp;
		value = limit;
	}
	else {
		if (value < -limit) {
			//temp = -limit / value;
			//value *= temp
			value = -limit;
		}
	}

	return value;
}

///////////////////////////////////////////////////////////////////////////////

/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : STM32x_svpwm_ics.c
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : ICS current reading and PWM generation module
* SVPWM_IcsCalcDutyCycles
********************************************************************************/
void mcFocSVPWM_ST2(LP_MC_FOC lpFoc) // +++
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

	if (Y < 0) {
		if (Z < 0) {
			sector = 5;
		}
		else {
			if (X > 0) {
				sector = 3;
			}
			else {
				sector = 4;
			}
		}
	}
	else {
		if (Z < 0) {
			if (X > 0) {
				sector = 1;
			}
			else {
				sector = 6;
			}
		}
		else {
			sector = 2;
		}
	}

	switch (sector) {
	case 1:
	case 4:
		hTimePhA = T_4 + ((T_2 + X - Z) * 0.5f);
		hTimePhB = hTimePhA + Z;
		hTimePhC = hTimePhB - X;
		break;

	case 2:
	case 5:
		hTimePhA = T_4 + ((T_2 + Y - Z) * 0.5f);
		hTimePhB = hTimePhA + Z;
		hTimePhC = hTimePhA - Y;
		break;

	case 3:
	case 6:
		hTimePhA = T_4 + ((T_2 + Y - X) * 0.5f);
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

/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : STM32x_svpwm_ics.c
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : ICS current reading and PWM generation module
* SVPWM_IcsCalcDutyCycles
********************************************************************************/
void mcFocSVPWM_ST1(LP_MC_FOC lpFoc) // +
{
	int bSector;
	float wX, wY, wZ;
	float wUAlpha, wUBeta;
	float hTimePhA, hTimePhB, hTimePhC;

	wUAlpha = +(lpFoc->Valpha * T_SQRT3);
	wUBeta = -(lpFoc->Vbeta  * T);

	wX = wUBeta;
	wY = (wUBeta + wUAlpha) * 0.5f;
	wZ = (wUBeta - wUAlpha) * 0.5f;

	if (wY<0) {
		if (wZ<0) {
			bSector = 5;
		}
		else {// wZ >= 0
			if (wX <= 0) {
				bSector = 4;
			}
			else {// wX > 0
				bSector = 3;
			}
		}
	}
	else { // wY > 0
		if (wZ >= 0) {
			bSector = 2;
		}
		else { // wZ < 0
			if (wX <= 0) {
				bSector = 6;
			}
			else { // wX > 0
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

void mcFocSVPWM0(LP_MC_FOC lpFoc)
{
	float Uref1, Uref2, Uref3;
	float X, Y, Z;
	float t_1, t_2;
	float t1, t2, t3;
	float PWM1, PWM2, PWM3;
	float Tpwm = 1.0;

	//mcInvClark(lpFoc);

	Uref1 = lpFoc->Vb;
	Uref2 = lpFoc->Va;
	Uref3 = lpFoc->Vc;

	if (Uref3 <= 0) {
		if (Uref2 <= 0) {
			lpFoc->sector = 2;
		} else {
			if (Uref1 <= 0) {
				lpFoc->sector = 6;
			} else {
				lpFoc->sector = 1;
			}
		}
	} else {
		if (Uref2 <= 0) {
			if (Uref1 <= 0) {
				lpFoc->sector = 4;
			} else {
				lpFoc->sector = 3;
			}
		} else {
			lpFoc->sector = 5;
		}
	}

	X = lpFoc->Vbeta;
	Y = 0.5f * (lpFoc->Vbeta + SQRT3 * lpFoc->Valpha);
	Z = 0.5f * (lpFoc->Vbeta - SQRT3 * lpFoc->Valpha);

	switch (lpFoc->sector) {
	case 1:
		t_1 = X;
		t_2 = -Z;
	 break;

	case 2:
		t_1 = Z;
		t_2 = Y;
	 break;

	case 3:
		t_1 = -Y;
		t_2 = X;
	 break;

	case 4:
		t_1 = -X;
		t_2 = Z;
	 break;

	case 5:
		t_1 = -Z;
		t_2 = -Y;
	 break;

	case 6:
		t_1 = Y;
		t_2 = -X;
	 break;
	}

	t1 = (Tpwm - t_1 - t_2) * 0.5;
	t2 = t1 + t_1;
	t3 = t2 + t_2;

	switch (lpFoc->sector) {
	case 1:
		PWM1 = t3;
		PWM2 = t2;
		PWM3 = t1;
	 break;

	case 2:
		PWM1 = -t2 + (t3 + t1); // ???
		PWM2 = t3;
		PWM3 = t1;
	 break;

	case 3:
		PWM1 = t1;
		PWM2 = t3;
		PWM3 = t2;
	 break;

	case 4:
		PWM1 = t1;
		PWM2 = -t2 + (t3 + t1); // ???;
		PWM3 = t3;
	 break;

	case 5:
		PWM1 = t2;
		PWM2 = t1;
		PWM3 = t3;
	 break;

	case 6:
		PWM1 = t3;
		PWM2 = t1;
		PWM3 = -t2 + (t3 + t1); // ???;
	 break;
	}

	float scale = 100;

	lpFoc->PWM1 = PWM1 * scale;
	lpFoc->PWM2 = PWM3 * scale;
	lpFoc->PWM3 = PWM2 * scale;

	//lpFoc->PWM1 = Uref1 * scale;
	//lpFoc->PWM2 = Uref2 * scale;
	//lpFoc->PWM3 = Uref3 * scale;

	lpFoc->X = X * 25;
	lpFoc->Y = Y * 25;
	lpFoc->Z = Z * 25;

	static int index = 0, sector_old = 0;
	if (sector_old != lpFoc->sector) {
		lpFoc->arrSector[index++] = lpFoc->sector;
		sector_old = lpFoc->sector;
	}
}

void mcFocSPWM(LP_MC_FOC lpFoc) // ???
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

	int Tpwm, T_2, T_4;

	Tpwm = 100;
	T_2 = Tpwm * 0.50f;

	mcInvClark(lpFoc);

	lpFoc->PWM1 = T_2 + (lpFoc->Va * -T_2);
	lpFoc->PWM2 = T_2 + (lpFoc->Vb * -T_2);
	lpFoc->PWM3 = T_2 + (lpFoc->Vc * -T_2);
}

/*
*	Triangular Third Harmonic Injection ( TTHI )
*	http://www.cnblogs.com/nixianmin/p/4791428.html
*/
void mcFocSVPWM_TTHI(LP_MC_FOC lpFoc) // +++
{
	float Tpwm = 100.0f * 0.5f;
	float vmin, vmax, Vcom, X, Y, Z, temp;

	if (lpFoc->Va > lpFoc->Vb) {
		vmax = lpFoc->Va;
		vmin = lpFoc->Vb;
	}
	else {
		vmax = lpFoc->Vb;
		vmin = lpFoc->Va;
	}

	if (lpFoc->Vc > vmax) {
		vmax = lpFoc->Vc;
	}
	else {
		if (lpFoc->Vc < vmin) {
			vmin = lpFoc->Vc;
		}
	}

	Vcom = (vmax + vmin) * 0.5f;
	X = (lpFoc->Va - Vcom) * 1.1547f;
	Y = (lpFoc->Vb - Vcom) * 1.1547f;
	Z = (lpFoc->Vc - Vcom) * 1.1547f;

	//////////////////////////////////////////////////////////////////////////////
	X = fLimitValue( X, 0.999 );
	Y = fLimitValue( Y, 0.999 );
	Z = fLimitValue( Z, 0.999 );
	//////////////////////////////////////////////////////////////////////////////

	lpFoc->PWM1 = Tpwm - (X * Tpwm); // Tpwm - (Vref * Tpwm);
	lpFoc->PWM2 = Tpwm - (Y * Tpwm);
	lpFoc->PWM3 = Tpwm - (Z * Tpwm);

	static int index = 0, sector_old = 0;
	if (sector_old != lpFoc->sector) {
		lpFoc->arrSector[index++] = lpFoc->sector;
		sector_old = lpFoc->sector;
	}
}

/*
*	Sinusoidal Third Harmonic Injection ( STHI )
*/
void mcFocSVPWM_STHI(LP_MC_FOC lpFoc) // ---  ???
{
	float Tpwm = 100.0f / 2.0f;
	float V, Vcom, Sin_W, X, Y, Z;

	V = sqrtf(lpFoc->Vd * lpFoc->Vd + lpFoc->Vq * lpFoc->Vq) / 1.5f;
	Sin_W = lpFoc->Va / V;

	/*
		"sinf(foc_deg_to_rad(lpFoc->angle * 3) - foc_deg_to_rad(90));" -> !!! foc_deg_to_rad(90) = ? if lpFoc.Vd != 0 !!!
	*/
	Vcom = -(V / 6) * svpwm_sin3_table[(int)(lpFoc->angle)];

	//Vcom = (V / 6.0f) * ( ( 3.0f * Sin_W ) + ( 4.0 * ( (3.0f/4.0)*Sin_W - (1.0f/4.0f)*Sin_W) ) );

	X = (lpFoc->Va + Vcom) * 1.1547f;
	Y = (lpFoc->Vb + Vcom) * 1.1547f;
	Z = (lpFoc->Vc + Vcom) * 1.1547f;

	lpFoc->PWM1 = Tpwm - (X * Tpwm);
	lpFoc->PWM2 = Tpwm - (Y * Tpwm);
	lpFoc->PWM3 = Tpwm - (Z * Tpwm);

	static int index = 0, sector_old = 0;
	if (sector_old != lpFoc->sector) {
		lpFoc->arrSector[index++] = lpFoc->sector;
		sector_old = lpFoc->sector;
	}
}

void svpwmInitSin3Table(void)
{
	int angle;
	for (angle = 0; angle <= 360; angle++) {
		svpwm_sin3_table[angle] = sinf(foc_deg_to_rad(3.0f * (float)angle) + foc_deg_to_rad(90));
	}
}