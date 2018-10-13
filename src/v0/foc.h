#ifndef __FOC_H__
#define __FOC_H__

#include "stm32f4xx_adc.h"
#include "stm32f4xx_dac.h"

#include "pid/pid.h"
#include "encoder.h"
#include "filter.h"

//#define FOC_ADC_Mode_Independent
#define FOC_ADC_DualMode_RegSimult_InjecSimult

#define SQRT3			( 1.732050807568877f )
#define divSQRT3		( 1.0f / SQRT3 )		// 0.57735026918f
#define SQRT3_DIV2		( SQRT3 * 0.5f )		// 0.86602540378f

//#define TWO_BY_SQRT3	( 2.0f * SQRT3 )

// ----------------------------------------------------------------------------
// ADC1
// Injected Channel:
#define PHASE_A_ADC_CHANNEL		ADC_Channel_8		// PB0
#define VOLT_FDBK_CHANNEL		ADC_Channel_11		// PC1

// Regular Channel:
#define AIN1_ADC_CHANNEL		ADC_Channel_14		// PC4
#define AIN2_ADC_CHANNEL		ADC_Channel_15		// PC5
// ----------------------------------------------------------------------------
// ADC2
// Injected Channel:
#define PHASE_B_ADC_CHANNEL		ADC_Channel_9		// PB1
#define AIN0_ADC_CHANNEL		ADC_Channel_12		// PC2
// ----------------------------------------------------------------------------
#define SAMPLING_TIME_CK		ADC_SampleTime_15Cycles

#define ARRAYSIZE				( 2 * 4 )
#define ADC1_DR					((uint32_t)0x4001204C)

typedef struct {
	float angle, fSinAngle, fCosAngle;

	int enable, main_state, sector;

	int16_t current_a, current_b;
	int16_t current_a_offset, current_b_offset;

	float Ia, Ib;
	float Ialpha, Ibeta;

	float Id, Iq, Is;
	float Id_des, Iq_des;
	float Id_des_filter, Iq_des_filter;

	PID pid_d, pid_q;

	float Vd, Vq;

	float Valpha, Vbeta;
	float Va, Vb, Vc;

	/*
	 * Chapter 2 - Inverter Control with Space Vector Modulation
	 * https://www.springer.com/cda/content/document/cda_downloaddocument/9783662469149-c2.pdf?SGWID=0-0-45-1507483-p177337582
	 */
	float Ubus;		// Const: 320 VDC
	float Us;		// Const: 200 VAC
	float Usmax;	// |Usmax| = 2/3 * Udc = 2/3 * 320VDC = 213.33
	float m;		// Modulation ratio: m = |Us| / Usmax = 200 / 213.33 = 0.9375

	int PWM1, PWM2, PWM3;

	float fMaxRPM, fMaxRPMforCW, fMaxRPMforCCW;

	float f_rpm_m, f_rpm_m_filtered_value;
	float f_rpm_t, f_rpm_t_filtered_value;
	float f_rpm_mt, f_rpm_mt_filtered_value;
	float f_rpm_mt_temp, f_rpm_mt_temp_filtered_value;

} MC_FOC, *LP_MC_FOC;

void focInit(LP_MC_FOC lpFocExt);
void mcFocSetAngle(LP_MC_FOC lpFoc, int angle);
void mcFocCalcCurrent(LP_MC_FOC lpFoc);

void mcClark(LP_MC_FOC lpFoc);
void mcPark(LP_MC_FOC lpFoc);
void mcInvPark(LP_MC_FOC lpFoc);
void mcInvClark(LP_MC_FOC lpFoc);
void mcUsrefLimit(LP_MC_FOC lpFoc);

void initDAC(void);
void ADC1_2_IRQHandler(void);
void adc_current_filter( uint16_t *current_a, uint16_t *current_b );

float fLimitValue(float value, float limit);

#endif
