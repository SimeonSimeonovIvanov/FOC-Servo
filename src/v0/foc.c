#include <string.h>
#include "foc.h"

//#define __POS_CONTROL__
#define __AI1_SET_SPEED__
//#define __POS_AND_SPEED_CONTROL__

const float P = 8196.0f;
const float Ts = 0.0025f;
const float fc = 3*4000000.0f;

extern uint32_t uwTIM10PulseLength;
extern int32_t sp_counter;

static volatile int32_t sp_pos, sp_update_counter = 0;
static volatile int32_t arrSpPos[10];
static volatile int32_t pv_pos = 0;
static volatile LP_MC_FOC lpFoc;

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

#ifdef __AI1_SET_SPEED__
	pidInit_test( &pidSpeed, 4.5, .20, 0.0, 0 );
	pidSetOutLimit_test( &pidSpeed, 1575, -1575 );
	pidSetIntegralLimit_test( &pidSpeed, 1575 );

	/*pidInit( &pidSpeed, 12.0f, 0.5f, 0.0f, 1.001f );
	pidSetOutLimit( &pidSpeed, 0.999f, -0.999f );
	pidSetIntegralLimit( &pidSpeed, 1.0f );
	pidSetInputRange( &pidSpeed, 3000 );*/
#endif

	///////////////////////////////////////////////////////////////////////////

#ifdef __POS_CONTROL__
	pidInit( &pidPos, 0.8f, 0.0005f, 0.0f, 0.001f );
	pidSetOutLimit( &pidPos, 0.999f, -0.999f );
	pidSetIntegralLimit( &pidPos, 0.2f );
	pidSetInputRange( &pidPos, 500 );
#endif

	///////////////////////////////////////////////////////////////////////////

#ifdef __POS_AND_SPEED_CONTROL__
	pidInit_test( &pidSpeed, 3.5f, 0.2f, 0, 0 );
	pidSetOutLimit_test( &pidSpeed, 1375, -1375 );
	pidSetIntegralLimit_test( &pidSpeed, 1375 );

	pidInit( &pidPos, 1.0f, 0.0f, 0.0f, 1.001f );
	pidSetOutLimit( &pidPos, 0.999f, -0.999f );
	pidSetIntegralLimit( &pidPos, 0.0f );
	pidSetInputRange( &pidPos, 4000 );
#endif

	///////////////////////////////////////////////////////////////////////////
	pidInit( &lpFoc->pid_d, 0.09f, 0.0009f, 0.0f, 1.00006f );
	pidSetOutLimit( &lpFoc->pid_d, 0.99f, -0.999f );
	pidSetIntegralLimit( &lpFoc->pid_d, 0.25f );
	pidSetInputRange( &lpFoc->pid_d, 2047.0f );

	pidInit( &lpFoc->pid_q, 0.09f, 0.0009f, 0.0f, 1.00006f );
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

void ADC_IRQHandler( void )
{
	volatile static uint16_t angle = 0;
	static volatile int32_t counter_pos_reg = 0, counter_get_speed = 0, counter_speed_reg = 0;

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
		} else {
			lpFoc->f_rpm_mt = 0.0f;
		}

		counter_get_speed = 0;

		/*if( lpFoc->f_rpm_mt < 10.0f && lpFoc->f_rpm_mt > -10.0f ) {
			pidSpeed.kp = 2.5;
		} else {
			pidSpeed.kp = 5.5;
		}*/
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

			//if( rpm_t > 10.0f || rpm_t < -10.0f ) {
				//lpFoc->f_rpm_mt = rpm_t;
				//lpFoc->f_rpm_mt = 2.0f * ( lpFoc->f_rpm_m * lpFoc->f_rpm_t ) / ( lpFoc->f_rpm_m + lpFoc->f_rpm_t );
			//}
		}

		uwTIM10PulseLength_old = uwTIM10PulseLength;
		enc_delta_old = enc_delta;
	}

	pv_pos = iEncoderGetAbsPos();
	sp_pos = sp_counter;

	static volatile float arrSpeedSP[10], arrSpeedFB[10];
	volatile float fb_speed_filter;

	pv_speed = lpFoc->f_rpm_mt;
	fb_speed_filter = ffilter( (float)pv_speed, arrSpeedFB, 4 );

	if( lpFoc->enable ) {
#ifdef __POS_CONTROL__
		if( 8 == ++counter_pos_reg ) {
			lpFoc->Iq_des = 1370.0f * pidTask( &pidPos, (float)sp_pos, (float)pv_pos );
			counter_pos_reg = 0;
		}
#endif

#ifdef __AI1_SET_SPEED__
		if( 4 == ++counter_speed_reg ) {
			sp_speed = 2.0f * ( ai0_filtered_value - 2047.0f );
			if( ( GPIO_ReadInputData( GPIOB ) & GPIO_Pin_13 ) ? 1 : 0 ) {
				sp_speed = -sp_speed;
			}

			lpFoc->Iq_des = pidTask_test( &pidSpeed, sp_speed, fb_speed_filter );
			//lpFoc->Iq_des = 1575.0f * pidTask( &pidSpeed, sp_speed, pv_speed );

			counter_speed_reg = 0;
		}
#endif

#ifdef __POS_AND_SPEED_CONTROL__
		/*static uint16_t counter01 = 0, counter02 = 0;

		if( ++counter01 == 16000 ) {
			if( counter02 < 60 ) {
				sp_counter = 137 * counter02;
				counter02 += 1;
			} else {
				counter02 = 0;
				sp_counter = 0;
				TIM2->CNT = pv_pos = 0;
			}
			counter01 = 0;
		}*/

		if( 8 == ++counter_pos_reg ) {
			static volatile float arrPosSP[10] = { 0 };
			volatile float sp_pos_temp;

			sp_pos_temp = sp_pos;ffilter( (float)sp_pos, arrPosSP, 3 );

			sp_speed = 3000.0f * pidTask( &pidPos, (float)sp_pos_temp, (float)pv_pos );
			counter_pos_reg = 0;
		}

		if( 4 == ++counter_speed_reg ) {
			float sp_speed_filter;

			sp_speed_filter = sp_speed;
			//sp_speed_filter = ffilter( (float)sp_speed, arrSpeedSP, 2 );

			lpFoc->Iq_des = pidTask_test( &pidSpeed, sp_speed_filter, fb_speed_filter );

			counter_speed_reg = 0;
		}
#endif
	} else {
		pidTask_test( &pidSpeed, 0, 0 );
		pidTask_test( &pidPos, 0, 0 );

		counter_speed_reg = 0;
		counter_pos_reg = 0;
	}

	/*static float dt = 1.0f/16000.0f;
	const  float freq = 500.0f;
	static float freqt = 1.0f / freq;
	static float t = 0;

	float a = 0.5 * sinf( (2.0f * 3.14f * freq) * t );
	t += dt;
	if( t >= freqt ) {
		t = 0;
	}*/

	///////////////////////////////////////////////////////////////////////////
	//lpFoc->Iq_des = ( ai0_filtered_value - 2047.0f ) * 2.0f;
	//lpFoc->Iq_des = 500;
	//lpFoc->Iq_des = 0;
	///////////////////////////////////////////////////////////////////////////
	//angle = readSanyoWareSaveEncoder();
	angle = readRawEncoderWithUVW();
	mcFocSetAngle( lpFoc, angle );
	mcFocCalcCurrent( lpFoc );
	//angle = 11.377f * encoderAddOffset( angle * 0.0878906f, 5 );
	///////////////////////////////////////////////////////////////////////////
	mcClark( lpFoc );
	mcPark( lpFoc );
	///////////////////////////////////////////////////////////////////////////
	if( !lpFoc->enable ) {
		lpFoc->Id_des = 0; lpFoc->Id = 0;
		lpFoc->Iq_des = 0; lpFoc->Iq = 0;
	}
	///////////////////////////////////////////////////////////////////////////
	lpFoc->Vd = pidTask( &lpFoc->pid_d, lpFoc->Id_des, lpFoc->Id );
	lpFoc->Vq = pidTask( &lpFoc->pid_q, lpFoc->Iq_des, lpFoc->Iq );
	///////////////////////////////////////////////////////////////////////////
	mcInvPark( lpFoc );
	mcInvClark( lpFoc );
	///////////////////////////////////////////////////////////////////////////
	//mcFocSVPWM_ST2_TTHI( lpFoc );
	//mcFocSVPWM0_TTHI( lpFoc );
	mcFocSVPWM_TTHI(lpFoc);
	///////////////////////////////////////////////////////////////////////////
	TIM_SetCompare1( TIM1, lpFoc->PWM1 );
	TIM_SetCompare2( TIM1, lpFoc->PWM2 );
	TIM_SetCompare3( TIM1, lpFoc->PWM3 );
	///////////////////////////////////////////////////////////////////////////
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed*0.5 + 2047, lpFoc->f_rpm_mt_temp_filtered_value*0.5 + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed*0.5 + 2047, lpFoc->f_rpm_mt_filtered_value*0.5 + 2047 );
	DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Ia + 2047, fb_speed_filter*0.5 + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed + 2047, ( sp_pos - pv_pos ) + 2047 );

	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed + 2047, lpFoc->f_rpm_mt + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed + 2047, lpFoc->angle + 2047 );

	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Is + 2047, lpFoc->f_rpm_mt_filtered_value + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Id + 2047, lpFoc->Iq + 2047 );

	GPIO_ResetBits( GPIOB, GPIO_Pin_2 );
}

void adc_current_filter( uint16_t *current_a, uint16_t *current_b )
{
	static int arrIa[10]={0}, arrIb[10]={0};

	arrIa[9] = arrIa[8];	arrIa[8] = arrIa[7];
	arrIa[7] = arrIa[6];	arrIa[6] = arrIa[5];
	arrIa[5] = arrIa[4];	arrIa[4] = arrIa[3];
	arrIa[3] = arrIa[2];	arrIa[2] = arrIa[1];
	arrIa[1] = arrIa[0];	arrIa[0] = *current_a;

	arrIb[9] = arrIb[8];	arrIb[8] = arrIb[7];
	arrIb[7] = arrIb[6];	arrIb[6] = arrIb[5];
	arrIb[5] = arrIb[4];	arrIb[4] = arrIb[3];
	arrIb[3] = arrIb[2];	arrIb[2] = arrIb[1];
	arrIb[1] = arrIb[0];	arrIb[0] = *current_b;

	//*current_a = ( arrIa[0] + arrIa[1] + arrIa[2] + arrIa[3] + arrIa[4] + arrIa[5] + arrIa[6]  + arrIa[7]  + arrIa[8] + arrIa[9] ) / 10;
	//*current_b = ( arrIb[0] + arrIb[1] + arrIb[2] + arrIb[3] + arrIb[4] + arrIb[5] + arrIb[6]  + arrIb[7]  + arrIb[8] + arrIb[9] ) / 10;

	*current_a = ( arrIa[0] + arrIa[1] + arrIa[2] ) / 3;
	*current_b = ( arrIb[0] + arrIb[1] + arrIb[2] ) / 3;
}
