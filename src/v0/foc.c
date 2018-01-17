#include <string.h>
#include "foc.h"

#define __IQ_CONTROL__              0
#define __POS_CONTROL__             1 // ---
#define __AI1_SET_SPEED__           2 // +++ ?
#define __POS_AND_SPEED_CONTROL__   3 // +++ ?

#define __CONTROL_MODE__            3

const float P = 8196.0f;
const float Ts = 0.0025f;
const float fc = 3*4000000.0f;

extern uint32_t uwTIM10PulseLength;
extern int16_t tim8_overflow;
extern int32_t sp_counter;

static volatile int32_t arrSpPos[10];
static volatile LP_MC_FOC lpFoc;

volatile int32_t pv_pos = 0, sp_pos = 0, pos_error = 0, sp_update_counter = 0, sp_pos_freq = 0;

volatile float sp_speed, pv_speed;
volatile int16_t enc_delta;

volatile uint16_t ai0, ai1, ADC_values[ARRAYSIZE];
volatile float ai0_filtered_value;

volatile PID pidPos, pidSpeed;

void focInit(LP_MC_FOC lpFocExt)
{
	lpFoc = lpFocExt;

	memset( lpFoc, 0, sizeof( MC_FOC ) );

	///////////////////////////////////////////////////////////////////////////

#if ( __CONTROL_MODE__ == __POS_CONTROL__ )
	pidInit( &pidPos, 0.2f, 0.0000f, 0.0f, 0.001f );
	pidSetOutLimit( &pidPos, 0.999f, -0.999f );
	pidSetIntegralLimit( &pidPos, 0.2f );
	pidSetInputRange( &pidPos, 2000 );
#endif

	///////////////////////////////////////////////////////////////////////////

#if ( __CONTROL_MODE__ == __POS_AND_SPEED_CONTROL__ || __CONTROL_MODE__ == __AI1_SET_SPEED__ )
	pidInit( &pidSpeed, 0.4f, 0.009f, 0.0f, 1.001f );
	pidSetOutLimit( &pidSpeed, 0.999f, -0.999f );
	pidSetIntegralLimit( &pidSpeed, 0.99f );
	pidSetInputRange( &pidSpeed, 100 );
#endif

	///////////////////////////////////////////////////////////////////////////

#if ( __CONTROL_MODE__ == __POS_AND_SPEED_CONTROL__ )
	pidInit( &pidPos, 3.5f, 0.0f, 0.0f, 1.001f );
	pidSetOutLimit( &pidPos, 0.999f, -0.999f );
	pidSetIntegralLimit( &pidPos, 0.0f );
	pidSetInputRange( &pidPos, 20000 );

	//pidInit_test( &pidPos, 1.0f, 0.0f, 0.0f, 1.001f );
	//pidSetOutLimit_test( &pidPos, 3000.0f, -3000.0f );
#endif

	///////////////////////////////////////////////////////////////////////////
	pidInit( &lpFoc->pid_d, 0.12f, 0.0038f, 0.0f, 1.00006f );
	pidSetOutLimit( &lpFoc->pid_d, 0.99f, -0.999f );
	pidSetIntegralLimit( &lpFoc->pid_d, 0.25f );
	pidSetInputRange( &lpFoc->pid_d, 2047.0f );

	pidInit( &lpFoc->pid_q, 0.12f, 0.0038f, 0.0f, 1.00006f );
	pidSetOutLimit( &lpFoc->pid_q, 0.999f, -0.999f );
	pidSetIntegralLimit( &lpFoc->pid_q, 0.25f );
	pidSetInputRange( &lpFoc->pid_q, 2047.0f );
	///////////////////////////////////////////////////////////////////////////

	initHall();
	initEncoder();
	svpwmInit();
	initDAC();
}

void initDAC(void)
{
	DAC_InitTypeDef DAC_InitStruct;

	// Init DAC 1 & 2:
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit( &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_DAC, ENABLE );

	DAC_StructInit(&DAC_InitStruct );
	DAC_Init( DAC_Channel_1, &DAC_InitStruct );
	DAC_Init( DAC_Channel_2, &DAC_InitStruct );

	DAC_Cmd( DAC_Channel_1, ENABLE );
	DAC_Cmd( DAC_Channel_2, ENABLE );
}

void mcFocSetAngle(LP_MC_FOC lpFoc, int angle)
{
	lpFoc->angle = (float)angle;
	lpFoc->fSinAngle = fSinAngle(angle);
	lpFoc->fCosAngle = fCosAngle(angle);
}

void mcFocCalcCurrent(LP_MC_FOC lpFoc)
{
	lpFoc->Ia = -1.0f * (float)( lpFoc->current_a - lpFoc->current_a_offset );
	lpFoc->Ib = -1.0f * (float)( lpFoc->current_b - lpFoc->current_b_offset );
}

void mcClark(LP_MC_FOC lpFoc)
{
	lpFoc->Ialpha =  lpFoc->Ia;
	lpFoc->Ibeta = ( lpFoc->Ia + ( 2 * lpFoc->Ib ) ) * divSQRT3;
}

void mcPark(LP_MC_FOC lpFoc)
{
	lpFoc->Id = +lpFoc->Ialpha * lpFoc->fCosAngle + lpFoc->Ibeta * lpFoc->fSinAngle;
	lpFoc->Iq = -lpFoc->Ialpha * lpFoc->fSinAngle + lpFoc->Ibeta * lpFoc->fCosAngle;
}

void mcInvPark(LP_MC_FOC lpFoc)
{
	lpFoc->Valpha = lpFoc->Vd * lpFoc->fCosAngle - lpFoc->Vq * lpFoc->fSinAngle;
	lpFoc->Vbeta  = lpFoc->Vd * lpFoc->fSinAngle + lpFoc->Vq * lpFoc->fCosAngle;
}

void mcInvClark(LP_MC_FOC lpFoc)
{
	lpFoc->Vb = lpFoc->Vbeta;
	lpFoc->Va = ( -lpFoc->Vbeta + ( SQRT3 * lpFoc->Valpha ) ) * 0.5f;
	lpFoc->Vc = ( -lpFoc->Vbeta - ( SQRT3 * lpFoc->Valpha ) ) * 0.5f;
}

void mcUsrefLimit(LP_MC_FOC lpFoc)
{
	float Usref, temp;

	Usref = sqrtf( ( lpFoc->Vd * lpFoc->Vd ) + ( lpFoc->Vq * lpFoc->Vq ) );

	if( Usref > 0.999f ) {
		temp = 0.999f / Usref;
		lpFoc->Vd *= temp;
		lpFoc->Vq *= temp;
	} else {
		if( Usref < -0.999f ) {
			temp = -0.999f / Usref;
			lpFoc->Vd *= temp;
			lpFoc->Vq *= temp;
		}
	}
}

void ADC_IRQHandler( void )
{
	volatile static int32_t counter_pos_reg = 0, counter_get_speed = 0, counter_speed_reg = 0;
	volatile static uint16_t angle = 0, temp = 0;

	GPIO_SetBits( GPIOB, GPIO_Pin_2 );

	if( ADC_FLAG_JEOC & ADC1->SR ) {
#ifdef FOC_ADC_Mode_Independent
		lpFoc->current_a = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_1 );
		lpFoc->current_b = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_2 );
#endif

#ifdef FOC_ADC_DualMode_RegSimult_InjecSimult
		lpFoc->current_a = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_1 );
		lpFoc->vbus_voltage = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_2 );
#endif
	}

	if( ADC_FLAG_JEOC & ADC2->SR ) {
#ifdef FOC_ADC_Mode_Independent
		dc_voltage = ADC_GetInjectedConversionValue( ADC2, ADC_InjectedChannel_1 );
		ai0 = ADC_GetInjectedConversionValue( ADC2, ADC_InjectedChannel_2 );

		if( !(ADC_FLAG_JEOC & ADC1->SR) ) {
			ADC_ClearITPendingBit( ADC2, ADC_IT_JEOC );
			return;
		}
#endif

#ifdef FOC_ADC_DualMode_RegSimult_InjecSimult
		lpFoc->current_b = ADC_GetInjectedConversionValue( ADC2, ADC_InjectedChannel_1 );
		ai0 = ADC_GetInjectedConversionValue( ADC2, ADC_InjectedChannel_2 );
#endif
	}

	ADC_ClearITPendingBit( ADC1, ADC_IT_JEOC );
	ADC_ClearITPendingBit( ADC2, ADC_IT_JEOC );

	adc_current_filter( &lpFoc->current_a, &lpFoc->current_b );

	if( !lpFoc->main_state ) {
		return;
	}

	enc_delta = -TIM4->CNT;

	if( 40 == ++counter_get_speed ) {
		volatile float rpm_m, rpm_t;

		TIM4->CNT = 0;

		rpm_m = (float)enc_delta;
		rpm_t = (float)uwTIM10PulseLength;
		if( enc_delta < 0 ) {
			rpm_t = -rpm_t;
		}

		lpFoc->f_rpm_m = 60.0f * ( rpm_m / ( P * Ts ) );

		if( rpm_t ) {
			lpFoc->f_rpm_t = 60.0f * ( fc / ( P * rpm_t ) );
		} else {
			lpFoc->f_rpm_t = 0.0f;
		}

		if( lpFoc->f_rpm_m + lpFoc->f_rpm_t ) {
			lpFoc->f_rpm_mt = 2.0f * ( lpFoc->f_rpm_m * lpFoc->f_rpm_t ) / ( lpFoc->f_rpm_m + lpFoc->f_rpm_t );
			//lpFoc->f_rpm_mt = 0.5f * ( lpFoc->f_rpm_m * lpFoc->f_rpm_t + lpFoc->f_rpm_m + lpFoc->f_rpm_t );
		} else {
			lpFoc->f_rpm_mt = 0.0f;
		}

		counter_get_speed = 0;
	} else {
		static int16_t enc_delta_old = 0, uwTIM10PulseLength_old = 0;

		if( ( enc_delta != enc_delta_old ) || ( uwTIM10PulseLength != uwTIM10PulseLength_old ) ) {
			float rpm_t = (float)uwTIM10PulseLength;

			if( rpm_t ) {
				rpm_t = 60.0f * ( fc / ( P * rpm_t ) );

				if( lpFoc->f_rpm_mt < 0 ) {
					rpm_t = -rpm_t;
				}

				lpFoc->f_rpm_mt = 2.0f * ( lpFoc->f_rpm_m * rpm_t ) / ( lpFoc->f_rpm_m + rpm_t );
			} else {
				rpm_t = 0.0f;
			}
		}

		uwTIM10PulseLength_old = uwTIM10PulseLength;
		enc_delta_old = enc_delta;
	}

	static volatile float arrSpeedSP[10] = { 0 }, arrSpeedFB[10] = { 0 };
	volatile float pv_speed_filter;

	pv_speed = lpFoc->f_rpm_mt;
	pv_speed_filter = ffilter( (float)pv_speed, arrSpeedFB, 4 );

	sp_counter = ( ( 0xffff * tim8_overflow ) + TIM8->CNT  ) * 20;

	pv_pos = iEncoderGetAbsPos();
	sp_pos = sp_counter;
	pos_error = sp_pos - pv_pos;

#if ( __CONTROL_MODE__ == __AI1_SET_SPEED__ )
	sp_speed = 2.0f * ( ai0_filtered_value - 2047.0f );
	if( ( GPIO_ReadInputData( GPIOB ) & GPIO_Pin_13 ) ? 1 : 0 ) {
		sp_speed = -sp_speed;
	}
#endif

	if( lpFoc->enable ) {
#if ( __CONTROL_MODE__ == __POS_CONTROL__ )
		if( 1 == ++counter_pos_reg ) {
			lpFoc->Iq_des = 1575.0f * pidTask( &pidPos, (float)sp_pos, (float)pv_pos );
			counter_pos_reg = 0;
		}
#endif

		if( 40 == ++temp) {
			static int32_t new = 0, old = 0;

			new = sp_pos;
			sp_pos_freq = new - old;
			old = new;

			temp = 0;
		}

#if ( __CONTROL_MODE__ == __POS_AND_SPEED_CONTROL__ )
		static int32_t counter01 = 0, counter02 = 0;

		if( ++counter01 == 8 ) {
			counter02 += 15;( ai0_filtered_value - 2047.0f ) * ( 1.0f / 10.0f );
			counter01 = 0;
		}

		//sp_pos = ai0_filtered_value*4;
		//sp_pos = counter02;

		if( 8 == ++counter_pos_reg ) {
			//static volatile float arrPosSP[10] = { 0 };
			static float sp_pos_old = 0;
			volatile float sp_pos_temp;
			float d_sp_pos;
			float pid_out;

			//float sp_pos_rpm  = ((60.0f * sp_pos_freq) * (400.0f / 8196.0f)); // Ts = 0.005 ms.

			sp_pos_temp = sp_pos; //ffilter( (float)sp_pos, arrPosSP, 2 );

			d_sp_pos = sp_pos_temp - sp_pos_old;
			sp_pos_old = sp_pos_temp;

			//sp_speed = ( 0.015f * d_sp_pos ) + pidTask_test( &pidPos, (float)sp_pos_temp, (float)pv_pos );
			pid_out = ( 0.00001f * d_sp_pos ) + pidTask( &pidPos, (float)sp_pos_temp, (float)pv_pos );
			//pid_out = ( 0.000125f * sp_pos_rpm ) + pidTask( &pidPos, (float)sp_pos_temp, (float)pv_pos );

			if( pid_out > 1.0f ) pid_out = 1.0f;
			if( pid_out < -1.0f ) pid_out = -1.0f;

			sp_speed = 3000.0f * pid_out;

			counter_pos_reg = 0;
		}
#endif

#if ( __CONTROL_MODE__ == __POS_AND_SPEED_CONTROL__ || __CONTROL_MODE__ == __AI1_SET_SPEED__ )
		if( 4 == ++counter_speed_reg ) {
			static float old_sp_speed = 0;
			float d_sp_speed;

			//static volatile float arrSpeedSP[10] = { 0 };
			//volatile float sp_speed_temp;
			//sp_speed_temp = ffilter( (float)sp_speed, arrSpeedSP, 5 );

			if( sp_speed > +3500.0f ) sp_speed = +3500.0f;
			if( sp_speed < -3500.0f ) sp_speed = -3500.0f;

			if( pv_speed < 15.0f && pv_speed > -15.0f ) {
				pidSpeed.kp = 0.25f;
				pidPos.kp = 2.0f;
			} else {
				pidSpeed.kp = 0.40f;
				pidPos.kp = 4.0f;
			}

			d_sp_speed = sp_speed - old_sp_speed;
			old_sp_speed = sp_speed;

			//lpFoc->Iq_des = ( 50.5 * d_sp_speed ) + pidTask_test( &pidSpeed, sp_speed, pv_speed_filter );
			lpFoc->Iq_des = 1575.0f * ( ( 0.001 * d_sp_speed ) + pidTask( &pidSpeed, sp_speed, pv_speed_filter ) );

			counter_speed_reg = 0;
		}
#endif
	} else {
		tim8_overflow = 0;
		TIM8->CNT = 0;

		counter_speed_reg = 0;
		counter_pos_reg = 0;
		sp_speed = 0;
		sp_pos = 0;

		lpFoc->Id_des = lpFoc->Id = 0;
		lpFoc->Vd = lpFoc->Vq = 0.0f;

		lpFoc->pid_d.sumError = 0.0f;
		lpFoc->pid_q.sumError = 0.0f;
		pidSpeed.sumError = 0.0f;
		pidPos.sumError = 0.0f;
	}
	///////////////////////////////////////////////////////////////////////////
	/*static float dt = 1.0f/16000.0f;
	const  float freq = 500.0f;
	static float freqt = 1.0f / freq;
	static float t = 0;

	float a = 0.01 * sinf( (2.0f * 3.14f * freq) * t );
	t += dt;
	if( t >= freqt ) {
		t = 0;
	}*/
	///////////////////////////////////////////////////////////////////////////
#if ( __CONTROL_MODE__ == __IQ_CONTROL__ )
	lpFoc->Iq_des = ( ai0_filtered_value - 2047.0f );
	//lpFoc->Iq_des = 500;
	//lpFoc->Iq_des = 0;
#endif
	///////////////////////////////////////////////////////////////////////////
	//angle = readSanyoWareSaveEncoder();
	angle = readRawEncoderWithUVW();
	mcFocSetAngle( lpFoc, angle );
	mcFocCalcCurrent( lpFoc );
	///////////////////////////////////////////////////////////////////////////
	mcClark( lpFoc );
	mcPark( lpFoc );
	///////////////////////////////////////////////////////////////////////////
	if( lpFoc->enable ) {
		static volatile float temp = 0;

		FirstOrderLagFilter( &temp, lpFoc->Iq_des, 0.95 );
		lpFoc->Iq_des = temp;

		if( lpFoc->Iq_des >  1575.0f ) lpFoc->Iq_des =  1575.0f;
		if( lpFoc->Iq_des < -1575.0f ) lpFoc->Iq_des = -1575.0f;

		lpFoc->Vd = pidTask( &lpFoc->pid_d, lpFoc->Id_des, lpFoc->Id );
		lpFoc->Vq = pidTask( &lpFoc->pid_q, lpFoc->Iq_des, lpFoc->Iq );
	}
	///////////////////////////////////////////////////////////////////////////
	mcUsrefLimit(lpFoc);
	///////////////////////////////////////////////////////////////////////////
	mcInvPark( lpFoc );
	mcInvClark( lpFoc );
	///////////////////////////////////////////////////////////////////////////
	//mcFocSVPWM_ST2_TTHI( lpFoc );
	//mcFocSVPWM0_TTHI( lpFoc );
	mcFocSVPWM_TTHI( lpFoc );
	//mcFocSVPWM_STHI( lpFoc );
	///////////////////////////////////////////////////////////////////////////
	TIM_SetCompare1( TIM1, lpFoc->PWM1 );
	TIM_SetCompare2( TIM1, lpFoc->PWM2 );
	TIM_SetCompare3( TIM1, lpFoc->PWM3 );
	///////////////////////////////////////////////////////////////////////////
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed * 0.5f + 2047, lpFoc->f_rpm_mt_temp_filtered_value * 0.5f + 2047 );
	DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed * 0.5f + 2047, lpFoc->f_rpm_mt_filtered_value * 0.5f + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed * 0.5f + 2047, lpFoc->f_rpm_mt * 0.5f + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed * 0.5f + 2047, pv_speed_filter * 0.5f + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed * 0.5f + 2047, lpFoc->angle + 2047 );

	//DAC_SetDualChannelData( DAC_Align_12b_R, (sp_pos - pv_pos) * 1.0f + 2047, pv_speed_filter * 0.5f + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_pos * 0.5f + 2047, pv_pos * 0.5f + 2047 );

	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Is + 2047, lpFoc->f_rpm_mt_filtered_value + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Id + 2047, lpFoc->Iq + 2047 );

	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Ia + 2047, pv_speed_filter*0.5 + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Iq + 2047, pv_speed_filter*0.5 + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Iq + 2047, sp_speed*0.5 + 2047);

	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Is*0.9f + 2047, lpFoc->Iq_des*0.9f + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Iq*0.9f + 2047, lpFoc->Iq_des*0.9f + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Ia * 0.5f + 2047, lpFoc->Ib * 0.5f + 2047 );

	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->PWM1, lpFoc->PWM2 );

	GPIO_ResetBits( GPIOB, GPIO_Pin_2 );
}

void adc_current_filter( uint16_t *current_a, uint16_t *current_b )
{
	static int32_t arrIa[30]={0}, arrIb[30]={0};

	/*arrIa[15] = arrIa[14];	arrIa[14] = arrIa[13];
	arrIa[13] = arrIa[12];	arrIa[12] = arrIa[11];
	arrIa[11] = arrIa[10];	arrIa[10] = arrIa[9];
	arrIa[9] = arrIa[8];	arrIa[8] = arrIa[7];
	arrIa[7] = arrIa[6];	arrIa[6] = arrIa[5];*/
	arrIa[5] = arrIa[4];	arrIa[4] = arrIa[3];
	arrIa[3] = arrIa[2];	arrIa[2] = arrIa[1];
	arrIa[1] = arrIa[0];	arrIa[0] = *current_a;

	/*arrIb[15] = arrIb[14];	arrIb[14] = arrIb[13];
	arrIb[13] = arrIb[12];	arrIb[12] = arrIb[11];
	arrIb[11] = arrIb[10];	arrIb[10] = arrIb[9];
	arrIb[9] = arrIb[8];	arrIb[8] = arrIb[7];
	arrIb[7] = arrIb[6];	arrIb[6] = arrIb[5];*/
	arrIb[5] = arrIb[4];	arrIb[4] = arrIb[3];
	arrIb[3] = arrIb[2];	arrIb[2] = arrIb[1];
	arrIb[1] = arrIb[0];	arrIb[0] = *current_b;

	*current_a = (int32_t)(
		arrIa[0] + arrIa[1] + arrIa[2] + arrIa[3]// + arrIa[4]// +
		//arrIa[5] + arrIa[6] + arrIa[7]  + arrIa[8] + arrIa[9]// +
		//arrIa[10]  + arrIa[11] + arrIa[12] + arrIa[13] + arrIa[14] + arrIa[15]
	) * ( 1.0f / 4.0f );
	*current_b = (int32_t)(
		arrIb[0] + arrIb[1] + arrIb[2] + arrIb[3]// + arrIb[4]// +
		//arrIb[5] + arrIb[6]  + arrIb[7] + arrIb[8] + arrIb[9]// +
		//arrIb[10] + arrIb[11]  + arrIb[12] + arrIb[13] + arrIb[14] + arrIb[15]
	) * ( 1.0f / 4.0f );
}
