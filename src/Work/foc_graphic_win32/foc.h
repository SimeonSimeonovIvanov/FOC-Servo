#ifndef __FOC_H__
#define __FOC_H__

#define SQRT3			1.732050807568877f
#define divSQRT3		( 1.0f / SQRT3 )		// 0.57735026918f
#define SQRT3_DIV2		( SQRT3 * 0.5f )		// 0.86602540378f

//#define SQRT3_DIV2		( SQRT3 * 0.5f )		// 0.86602540378f
//#define TWO_BY_SQRT3	( 2.0f * SQRT3 )

#define foc_deg_to_rad( deg ) ( deg * ( 3.14159265359f / 180.0f ) )

typedef struct {
	float angle;
	float fSinAngle, fCosAngle;

	int sector;
	int arrSector[100];
	float Ia, Ib;

	float Ialpha, Ibeta;
	float Id, Iq;
	float Vd, Vq;
	float Valpha, Vbeta;
	float Va, Vb, Vc;

	int X, Y, Z;

	int PWM1, PWM2, PWM3;
} MC_FOC, *LP_MC_FOC;

void mcFoc(LP_MC_FOC lpFoc);
void mcFocSetAngle(LP_MC_FOC lpFoc, int angle);
void mcFocSetCurrent(LP_MC_FOC lpFoc, float Ia, float Ib);
void mcFocInitStruct(LP_MC_FOC lpFoc);

void mcClark(LP_MC_FOC lpFoc);
void mcPark(LP_MC_FOC lpFoc);
void mcInvPark(LP_MC_FOC lpFoc);
void mcInvClark(LP_MC_FOC lpFoc);
void mcUsrefLimit(LP_MC_FOC lpFoc);

void mcFocSVPWM_ST2(LP_MC_FOC lpFoc);
void mcFocSVPWM_ST1(LP_MC_FOC lpFoc);
void mcFocSVPWM0(LP_MC_FOC lpFoc);
void mcFocSPWM(LP_MC_FOC lpFoc);
void mcFocSVPWM_TTHI(LP_MC_FOC lpFoc);
void mcFocSVPWM_STHI(LP_MC_FOC lpFoc);

float fSinAngle(int angle);
float fCosAngle(int angle);
void svpwmInitSin3Table(void);

float fLimitValue(float value, float limit);

#endif