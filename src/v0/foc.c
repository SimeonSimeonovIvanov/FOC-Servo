#include <string.h>
#include "foc.h"

#define __IQ_CONTROL__              0
#define __AI1_SET_SPEED__           1 // +++ ?
#define __POS_AND_SPEED_CONTROL__   2 // +++ ?

#define __CONTROL_MODE__            __AI1_SET_SPEED__//__POS_AND_SPEED_CONTROL__

const float P = 8192.0f;
const float fc = 3*4000000.0f;

const int Ts_rep_counter = 20;
float Ts;

extern uint32_t uwTIM10PulseLength;
extern int16_t tim8_overflow;

float vel;

int32_t sp_pos, pv_pos, sp_counter;
float sp_speed, pv_speed;
int16_t enc_delta;

uint16_t ai0, ai1, ADC_values[ARRAYSIZE];
float ai0_filtered_value;

PID pidPos, pidSpeed;

static int32_t arrSpPos[10];
static volatile LP_MC_FOC lpFoc;

void focInit(LP_MC_FOC lpFocExt)
{
	lpFoc = lpFocExt;

	memset( lpFoc, 0, sizeof( MC_FOC ) );

	lpFoc->fMaxRPM = 3000.0f;

	lpFoc->Ubus = 320;
	lpFoc->Us = 200.0f;

	lpFoc->Usmax = ( 2.0f / 3.0f ) * lpFoc->Ubus;
	lpFoc->m = lpFoc->Us / lpFoc->Usmax;

	lpFoc->Id_des = 0.0f;

	///////////////////////////////////////////////////////////////////////////

	sp_pos = 0;
	pv_pos = 0;
	sp_speed = 0;
	pv_speed = 0;
	sp_counter = 0;

	///////////////////////////////////////////////////////////////////////////
	/*        EATON Wiring Manual | 2011
	 * "The rotation direction of a motor is always
	 * determined by directly looking at the drive
	 * shaft of the motor (from the drive end). On
	 * motors with two shaft ends, the driving end
	 * is denoted with D (= Drive), the non-driving
	 * end with N (= No drive)."
	 */
	lpFoc->fMaxRPMforCW  = 3000.00; // FWD - forward run ( +, clockwise rotation field )
	lpFoc->fMaxRPMforCCW = 3000.00; // REV - reverse run ( -, anticlockwise rotation field active )

	if( lpFoc->fMaxRPMforCW > lpFoc->fMaxRPM ) {
		lpFoc->fMaxRPMforCW = lpFoc->fMaxRPM;
	}

	if( lpFoc->fMaxRPMforCCW > lpFoc->fMaxRPM ) {
		lpFoc->fMaxRPMforCCW = lpFoc->fMaxRPM;
	}
	///////////////////////////////////////////////////////////////////////////

#if ( __CONTROL_MODE__ == __POS_AND_SPEED_CONTROL__ || __CONTROL_MODE__ == __AI1_SET_SPEED__ )
	pidInit( &pidSpeed, 1.1f, 0.0091f, 0.0f, 1.0f );

	pidSetOutLimit( &pidSpeed, 0.999f, -0.999f );
	pidSetIntegralLimit( &pidSpeed, 0.999f );
	pidSetInputRange( &pidSpeed, 200 );
#endif

#if ( __CONTROL_MODE__ == __POS_AND_SPEED_CONTROL__ )
	pidInit_test( &pidPos, 0.5f, 0.001f, 0.0f, 1.0f );

	pidSetIntegralLimit_test( &pidPos, 3000.0f );
	pidSetOutLimit_test( &pidPos, 3000.0f, -3000.0f );
#endif

	///////////////////////////////////////////////////////////////////////////
	pidInit( &lpFoc->pid_d, 0.12f, 0.0012f, 0.0f, 1.00006f );
	pidSetOutLimit( &lpFoc->pid_d, 0.99f, -0.999f );
	pidSetIntegralLimit( &lpFoc->pid_d, 0.25f );
	pidSetInputRange( &lpFoc->pid_d, 2047.0f );

	pidInit( &lpFoc->pid_q, 0.12f, 0.0012f, 0.0f, 1.00006f );
	pidSetOutLimit( &lpFoc->pid_q, 0.999f, -0.999f );
	pidSetIntegralLimit( &lpFoc->pid_q, 0.25f );
	pidSetInputRange( &lpFoc->pid_q, 2047.0f );
	///////////////////////////////////////////////////////////////////////////

	initHall();
	initEncoder();
	svpwmInit();
	initDAC();

	Ts = Ts_rep_counter * ( 1.0f / (float)svpwmGetFrq() );
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

void mcPMSM_QD_PID(LP_MC_FOC lpFoc)
{
	if( lpFoc->enable ) {
		if( lpFoc->Iq_des >  1900.0f ) lpFoc->Iq_des =  1575.0f;
		if( lpFoc->Iq_des < -1900.0f ) lpFoc->Iq_des = -1575.0f;

		//FirstOrderLagFilter( &lpFoc->Id_des_filter, lpFoc->Id_des, 0.99 );
		lpFoc->Id_des_filter = lpFoc->Id_des;

		FirstOrderLagFilter( &lpFoc->Iq_des_filter, lpFoc->Iq_des, 0.99 );
		//lpFoc->Iq_des_filter = lpFoc->Iq_des;

		lpFoc->Vd = pidTask( &lpFoc->pid_d, lpFoc->Id_des_filter, lpFoc->Id );
		lpFoc->Vq = pidTask( &lpFoc->pid_q, lpFoc->Iq_des_filter, lpFoc->Iq );
	} else {
		lpFoc->pid_d.sumError = 0.0f;
		lpFoc->pid_q.sumError = 0.0f;

		lpFoc->Id_des_filter = 0;
		lpFoc->Iq_des_filter = 0;

		lpFoc->Id_des = 0.0f;
		lpFoc->Iq_des = 0.0f;

		lpFoc->Vd = 0.0f;
		lpFoc->Vq = 0.0f;
	}
}

void mcUsrefLimit(LP_MC_FOC lpFoc)
{
	float limit = lpFoc->m;
	float Usref, scale;

	Usref = sqrtf( lpFoc->Vd * lpFoc->Vd + lpFoc->Vq * lpFoc->Vq );

	if( Usref > limit ) {
		scale = limit / Usref;
		lpFoc->Vd *= scale;
		lpFoc->Vq *= scale;
	}
}

void ADC_IRQHandler( void )
{
	static int32_t counter_pos_reg = 0, counter_speed_reg = 0;
	static int32_t counter_get_speed = 0;
	static float arrSpeedFB[10] = { 0 };
	static uint16_t angle = 0;

	GPIO_SetBits( GPIOB, GPIO_Pin_2 );

	if( ADC_FLAG_JEOC & ADC1->SR ) {
#ifdef FOC_ADC_Mode_Independent
		lpFoc->current_a = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_1 );
		lpFoc->current_b = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_2 );
#endif

#ifdef FOC_ADC_DualMode_RegSimult_InjecSimult
		lpFoc->current_a = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_1 );
		lpFoc->Ubus = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_2 );
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
		lpFoc->enable = 0;
		return;
	}

	enc_delta = TIM4->CNT;
	if( Ts_rep_counter == ++counter_get_speed ) {
		volatile float rpm_m, rpm_t;

		TIM4->CNT = 0;

		rpm_m = (float)enc_delta;
		rpm_t = (float)uwTIM10PulseLength;
		if( rpm_m < 0 ) {
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

		////////////////////////////////////////////
		static int16_t lastpos = 0;
		int16_t pos;

		pos = TIM3->CNT;
		vel = ( ( pos - lastpos + 4095 ) % 4096 );
		lastpos = pos;

		if( vel < 2000 ) vel = -vel;
		else vel = vel - 4095;

		vel = vel * ( 60.0f / ( 8192.0f * Ts ) );
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

	//lpFoc->f_rpm_mt *= 0.740;
	pv_speed = lpFoc->f_rpm_mt;
	pv_speed = ffilter( (float)pv_speed, arrSpeedFB, 2 );

	//sp_counter = ( ( 0xffff * tim8_overflow ) + TIM8->CNT );
	sp_counter = ( ( 0xffff * tim8_overflow ) + TIM8->CNT ) * 10;
	//sp_counter = ( ( ( 8192.0f - 1.0f ) / 2000.0f ) * (float)sp_counter ) * -1.0f;

	pv_pos = -iEncoderGetAbsPos();

#if ( __CONTROL_MODE__ == __AI1_SET_SPEED__ )
	sp_speed = 2.0f * ( ai0_filtered_value - 2047.0f );
	if( ( GPIO_ReadInputData( GPIOB ) & GPIO_Pin_13 ) ? 1 : 0 ) {
		sp_speed = -sp_speed;
	}
#endif

	if( lpFoc->enable ) {
#if ( __CONTROL_MODE__ == __POS_AND_SPEED_CONTROL__ )
		static int32_t counter01 = 0, counter02 = 0;

		if( ++counter01 == 32000 ) {
			counter02 += 1*8191;( ai0_filtered_value - 2047.0f ) * ( 1.0f / 10.0f );
			counter01 = 0;
		}

		//sp_pos = ai0_filtered_value - 2047.0f;
		//sp_pos = counter02;
		sp_pos = sp_counter;
#endif

#if ( __CONTROL_MODE__ == __POS_AND_SPEED_CONTROL__ )
		if( 1 == ++counter_pos_reg ) {
			static float sp_pos_old = 0;
			float delta_sp_pos;

			if( fabs( pidPos.error ) < 50.0f ) {
				//pidPos.kp = 0.5f;
			} else {
				//pidPos.kp = 1.0f;
			}

			delta_sp_pos = sp_pos - sp_pos_old;
			sp_pos_old = sp_pos;

			/*pid_out = ( 0.00f * d_sp_pos + 0.000 * d_speed_freq ) + pidTask( &pidPos, (float)sp_pos, (float)-pv_pos );
			if( pid_out > 1.0f ) pid_out = 1.0f;
			if( pid_out < -1.0f ) pid_out = -1.0f;
			sp_speed = -lpFoc->fMaxRPM * pid_out;*/

			sp_speed = ( delta_sp_pos * 5.0f ) + pidTask_test( &pidPos, (float)sp_pos, (float)pv_pos );

			counter_pos_reg = 0;
		}
#endif

#if ( __CONTROL_MODE__ == __POS_AND_SPEED_CONTROL__ || __CONTROL_MODE__ == __AI1_SET_SPEED__ )
		if( 1 == ++counter_speed_reg ) {
			if( sp_speed > +lpFoc->fMaxRPMforCW ) {
				sp_speed = +lpFoc->fMaxRPMforCW;
			}
			if( sp_speed < -lpFoc->fMaxRPMforCCW ) {
				sp_speed = -lpFoc->fMaxRPMforCCW;
			}

			if( fabs( pv_speed ) < 15.0f ) {
				//pidSpeed.kp = 0.35f;
			} else {
				//pidSpeed.kp = 0.50f;
			}

			lpFoc->Iq_des = 1575.0f * pidTask( &pidSpeed, sp_speed, -pv_speed );

			counter_speed_reg = 0;
		}
#endif
	} else {
		TIM_Cmd( TIM8, DISABLE );
		tim8_overflow = 0;
		TIM8->CNT = 0;

		counter_speed_reg = 0;
		counter_pos_reg = 0;
		sp_speed = 0;
		sp_pos = 0;

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
	//lpFoc->Iq_des = ( ai0_filtered_value - 2047.0f );
	lpFoc->Iq_des = 500;
	//lpFoc->Iq_des = 0;
#endif
	///////////////////////////////////////////////////////////////////////////
//#define __ACIM_VF_TEST__

#ifdef __ACIM_VF_TEST__
	sp_speed = 200;( ai0 - 2047 );// * ();

	static float limit = 1;

	/*if( lpFoc->Is >= 2200 ) {
		if( limit < sp_speed ) limit+= 1;
	} else {
		if( limit>1 ) limit -= 1;
	}*/

	if(sp_speed) {
		sp_speed = sp_speed - limit;
	} else {
		sp_speed = -sp_speed - limit;
	}

	static float angle_int = 0;
	float vf_q = ( fabs( sp_speed ) / 1385 );

	angle_int += ( ( 50.0f * vf_q ) / (float)svpwmGetFrq() );
	if( angle_int > 0.999 ) {
		angle_int = 0;
	}

	angle = (int)( ( angle_int * 360.0f ) * 11.375f );
	if( sp_speed < 0 ) {
		angle = 4095 - angle;
	}

#else
	//angle = readSanyoWareSaveEncoder();
	angle = readRawEncoderWithUVW();
#endif

	mcFocSetAngle( lpFoc, angle );
	mcFocCalcCurrent( lpFoc );
	///////////////////////////////////////////////////////////////////////////
	mcClark( lpFoc );
	mcPark( lpFoc );
	///////////////////////////////////////////////////////////////////////////
#ifdef __ACIM_VF_TEST__
	if( lpFoc->enable ) {
		lpFoc->Vd = 0;//mos_sp_speed * 0.;// * 1/(limit/3000);
		lpFoc->Vq = vf_q * ( 220.0f / 270.0f )*0.5;

		lpFoc->Vd = lpFoc->Vq;

		if(lpFoc->Vq < 0.05) {
			//lpFoc->Vq += 0.05f;
		}
	}
#else
	mcPMSM_QD_PID( lpFoc );
#endif
	///////////////////////////////////////////////////////////////////////////
	mcUsrefLimit( lpFoc );
	///////////////////////////////////////////////////////////////////////////
	mcInvPark( lpFoc );
	mcInvClark( lpFoc );
	///////////////////////////////////////////////////////////////////////////
	//mcFocSVPWM_ST2_TTHI( lpFoc );
	//mcFocSVPWM0_TTHI( lpFoc );

	//mcFocSPWM( lpFoc );
	mcFocSVPWM_TTHI( lpFoc );
	//mcFocSVPWM_STHI( lpFoc );

	//SPWM( lpFoc, (float)0.99f, angle);
	///////////////////////////////////////////////////////////////////////////
	TIM_SetCompare1( TIM1, lpFoc->PWM1 );
	TIM_SetCompare2( TIM1, lpFoc->PWM2 );
	TIM_SetCompare3( TIM1, lpFoc->PWM3 );
	///////////////////////////////////////////////////////////////////////////
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed * 0.5f + 2047, lpFoc->f_rpm_mt_temp_filtered_value * 0.5f + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed * 0.5f + 2047, lpFoc->f_rpm_mt_filtered_value * 0.5f + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed * 0.5f + 2047, lpFoc->f_rpm_mt * 0.5f + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed * 0.5f + 2047, pv_speed * 0.5f + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_speed * 0.5f + 2047, lpFoc->angle + 2047 );

	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->PWM2/4, lpFoc->angle );

	//DAC_SetDualChannelData( DAC_Align_12b_R, (sp_pos - pv_pos) * 1.0f + 2047, pv_speed_filter * 0.5f + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, sp_pos * 0.5f + 2047, pv_pos * 0.5f + 2047 );

	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Is + 2047, lpFoc->f_rpm_mt_filtered_value + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Id + 2047, lpFoc->Iq + 2047 );

	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Ia + 2047, pv_speed_filter*0.5 + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Iq + 2047, pv_speed_filter*0.5 + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Iq + 2047, sp_speed*0.5 + 2047);

	//DAC_SetDualChannelData( DAC_Align_12b_R,  lpFoc->PWM1, lpFoc->PWM2 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Iq*0.9f + 2047, lpFoc->Iq_des*0.9f + 2047 );

	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Iq*0.9f + 2047, lpFoc->Id*0.9f + 2047 );
	DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Ia * 0.5f + 2047, lpFoc->Ib * 0.5f + 2047 );

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
		arrIa[0] + arrIa[1] + arrIa[2] + arrIa[3] //+ arrIa[4] +
		//arrIa[5] + arrIa[6] + arrIa[7]  + arrIa[8] + arrIa[9]// +
		//arrIa[10]  + arrIa[11] + arrIa[12] + arrIa[13] + arrIa[14] + arrIa[15]
	) * ( 1.0f / 4.0f );
	*current_b = (int32_t)(
		arrIb[0] + arrIb[1] + arrIb[2] + arrIb[3] //+ arrIb[4] +
		//arrIb[5] + arrIb[6]  + arrIb[7] + arrIb[8] + arrIb[9]// +
		//arrIb[10] + arrIb[11]  + arrIb[12] + arrIb[13] + arrIb[14] + arrIb[15]
	) * ( 1.0f / 4.0f );
}

float fLimitValue(float value, float limit)
{
	//float temp;

	if (value > limit) {
		//temp = limit / value;
		//value *= temp;
		value = limit;
	} else {
		if (value < -limit) {
			//temp = -limit / value;
			//value *= temp
			value = -limit;
		}
	}

	return value;
}
