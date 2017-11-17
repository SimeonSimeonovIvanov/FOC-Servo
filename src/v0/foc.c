#include "foc.h"

//#define __POS_CONTROL__
#ifndef __POS_CONTROL__
	#define __AI1_SET_SPEED__
	#ifndef __AI1_SET_SPEED__
		#define __POS_AND_SPEED_CONTROL__
	#endif
#endif

extern int current_a_offset, current_b_offset;
extern int32_t sp_counter;
extern int main_state;
extern float f_rpm_m, f_rpm_t, f_rpm_mt;
extern float f_rpm_m_filtered_value, f_rpm_t_filtered_value, f_rpm_mt_filtered_value;

uint16_t current_a = 0, current_b = 0;

static LP_MC_FOC lpFoc;

uint16_t ai0, ai1;
uint16_t ADC_values[ARRAYSIZE];

int16_t enc_delta;

PID pidPos, pidSpeed;

void focInit(LP_MC_FOC lpFocExt)
{
	lpFoc = lpFocExt;

	memset( lpFoc, 0, sizeof( MC_FOC ) );

	///////////////////////////////////////////////////////////////////////////
#ifdef __AI1_SET_SPEED__
	//pidInit_test( &pidSpeed, 15, 5, 0, 0 );
	pidInit_test( &pidSpeed, 30, 7, 10, 0 );
	//pidInit_test( &pidSpeed, 20, 7, 2, 0 );

	pidSetOutLimit_test( &pidSpeed, 1375, -1375 );
	pidSetIntegralLimit_test( &pidSpeed, 170 );
#endif

#ifdef __POS_CONTROL__
	pidInit( &pidPos, 0.8f, 0.0005f, 0.0f, 0.001f );
	pidSetOutLimit( &pidPos, 0.999f, -0.999f );
	pidSetIntegralLimit( &pidPos, 0.2f );
	pidSetInputRange( &pidPos, 200 );
#endif

#ifdef __POS_AND_SPEED_CONTROL__
	pidInit_test( &pidSpeed, 10, 1, 0, 0 );
	pidSetOutLimit_test( &pidSpeed, 1375, -1375 );
	pidSetIntegralLimit_test( &pidSpeed, 50 );

	pidInit( &pidPos, 0.7f, 0.00f, 0.0f, 0.001f );
	pidSetOutLimit( &pidPos, 0.999f, -0.999f );
	pidSetIntegralLimit( &pidPos, 0.0f );
	pidSetInputRange( &pidPos, 400 );
#endif
	///////////////////////////////////////////////////////////////////////////
	pidInit( &lpFoc->pid_d, 0.7f, 0.001f, 0.0f, 1.00006f );
	pidSetOutLimit( &lpFoc->pid_d, 0.99f, -0.999f );
	pidSetIntegralLimit( &lpFoc->pid_d, 0.3f );
	pidSetInputRange( &lpFoc->pid_d, 2047.0f );

	pidInit( &lpFoc->pid_q, 0.7f, 0.001f, 0.0f, 1.00006f );
	pidSetOutLimit( &lpFoc->pid_q, 0.999f, -0.999f );
	pidSetIntegralLimit( &lpFoc->pid_q, 0.3f );
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

void mcFocSetCurrent(LP_MC_FOC lpFoc, float Ia, float Ib)
{
	lpFoc->Ia = Ia;
	lpFoc->Ib = Ib;
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

extern uint16_t uwTIM10PulseLength;
void ADC_IRQHandler( void )
{
	static int temp = 0;

	uint16_t angle;
	float Ia, Ib, sp_speed;

	GPIO_SetBits( GPIOB, GPIO_Pin_2 );

	if( ADC_FLAG_JEOC & ADC1->SR ) {
#ifdef FOC_ADC_Mode_Independent
		current_a = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_1 );
		current_b = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_2 );
#endif

#ifdef FOC_ADC_DualMode_RegSimult_InjecSimult
		current_a = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_1 );
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
		current_b = ADC_GetInjectedConversionValue( ADC2, ADC_InjectedChannel_1 );
		ai0 = ADC_GetInjectedConversionValue( ADC2, ADC_InjectedChannel_2 );
#endif
	}

	ADC_ClearITPendingBit( ADC1, ADC_IT_JEOC );
	ADC_ClearITPendingBit( ADC2, ADC_IT_JEOC );

	adc_current_filter( &current_a, &current_b );

	if( !main_state ) {
		return;
	}

	float P = 8000.0f;
	float Ts = 0.0024f;
	float fc = 4000000.0f;
	static float rpm_m, rpm_t;

	{
		static int32_t arrSpPos[10], counter = 0, counter1 = 0, counter2 = 0;
		static int32_t sp_pos, sp_update_counter = 0;
		static int32_t pv_pos = 0;

		static int32_t sp_counter_old = 0;
		static int32_t sp_counter_temp = 0;

#ifdef __POS_CONTROL__
		pv_pos = iEncoderGetAbsPos();
		sp_pos = sp_counter;

		if( sp_counter != pv_pos ) {

			if( sp_counter - sp_counter_old > 0 ) {
				sp_counter_temp += 10;
			}

			if( sp_counter - sp_counter_old < 0 ) {
				sp_counter_temp -= 10;
			}
		}

		sp_counter_old = sp_counter;

		if( ++sp_update_counter == 75 ) {
			sp_update_counter = 0;
			//sp_pos = sp_counter;
			//sp_pos = sp_counter_temp;
		}

		if( 1 == ++counter ) {
			lpFoc->Iq_des = 1370.0f * pidTask( &pidPos, (float)sp_pos, (float)pv_pos );
			counter = 0;
		}
#endif

		if( 40 == ++counter1 ) {
			enc_delta = -TIM4->CNT;
			TIM4->CNT = 0;

			rpm_m = (float)enc_delta;
			rpm_t = (float)uwTIM10PulseLength;

			float m1 = rpm_m, m2 = rpm_t;
			f_rpm_m = 60 * ( m1 / ( P * Ts ) );
			f_rpm_t = 60 * ( fc / ( P * m2 ) );

			if(f_rpm_m<0) f_rpm_t = -f_rpm_t;

			if( m2 ) {
				f_rpm_mt = 60 * ( ( fc * m1 ) / ( P * m2 ) );
			} else {
				f_rpm_mt = 0.0f;
			}

			if( 1 ) {
				if( f_rpm_m + f_rpm_t ) {
					f_rpm_mt = 2 * (f_rpm_m * f_rpm_t) / (f_rpm_m + f_rpm_t);
				} else {
					f_rpm_mt = 0.0f;
				}
			}

			///////////////////////////////////////////////////////////////////
			counter1 = 0;
		}

		if( 40 == ++counter2 ) {
#ifdef __AI1_SET_SPEED__

			sp_speed = ai0 - 2047;
			sp_speed /= 3;

			if( ( GPIO_ReadInputData( GPIOB ) & GPIO_Pin_13 ) ? 1 : 0 ) {
				sp_speed = -sp_speed;
			}

			/*if( !sp_speed || ( sp_speed < 50 && sp_speed > -50 ) ) {
				//sp_speed = 0.0f;
			} else {
				if( sp_speed>=0 ) {
					//sp_speed -= 10;
				} else {
				//sp_speed += 10;
				}
			}*/

			static int32_t arrSpeedSP[10];
			int32_t pv_speed;

			pv_speed = f_rpm_mt/3;enc_delta;(int)f_rpm_m_filtered_value/3;;f_rpm_m/3;enc_delta;

			arrSpeedSP[9] = arrSpeedSP[8];	arrSpeedSP[8] = arrSpeedSP[7];
			arrSpeedSP[7] = arrSpeedSP[6];	arrSpeedSP[6] = arrSpeedSP[5];
			arrSpeedSP[5] = arrSpeedSP[4];	arrSpeedSP[4] = arrSpeedSP[3];
			arrSpeedSP[3] = arrSpeedSP[2];	arrSpeedSP[2] = arrSpeedSP[1];
			arrSpeedSP[1] = arrSpeedSP[0];	arrSpeedSP[0] = sp_speed;

			sp_speed = ( arrSpeedSP[0] + arrSpeedSP[1] + arrSpeedSP[2] + arrSpeedSP[3] + arrSpeedSP[4] + arrSpeedSP[5] + arrSpeedSP[6]  + arrSpeedSP[7]  + arrSpeedSP[8] + arrSpeedSP[9] ) / 10;

			lpFoc->Iq_des = pidTask_test( &pidSpeed, sp_speed, pv_speed );
#endif

			counter2 = 0;
		}

#ifdef __POS_AND_SPEED_CONTROL__
		if( 1 == ++counter ) {
				sp_speed = 200 * pidTask( &pidPos, (float)sp_pos, (float)pv_pos );
				counter = 0;
		}

		if( 40 == ++counter1 ) {
			static int32_t enc_old = 0, dir = 0;
			int32_t enc;

			enc = iEncoderGetAbsPos();
			enc_delta = enc - enc_old;
			enc_old = enc;

			counter1 = 0;
		}

		if( 40 == ++counter2 ) {
			static int32_t arrSpeedSP[10];
			int32_t pv_speed;

			pv_speed = enc_delta;

			arrSpeedSP[9] = arrSpeedSP[8];	arrSpeedSP[8] = arrSpeedSP[7];
			arrSpeedSP[7] = arrSpeedSP[6];	arrSpeedSP[6] = arrSpeedSP[5];
			arrSpeedSP[5] = arrSpeedSP[4];	arrSpeedSP[4] = arrSpeedSP[3];
			arrSpeedSP[3] = arrSpeedSP[2];	arrSpeedSP[2] = arrSpeedSP[1];
			arrSpeedSP[1] = arrSpeedSP[0];	arrSpeedSP[0] = sp_speed;

			//sp_speed = ( arrSpeedSP[0] + arrSpeedSP[1] + arrSpeedSP[2] + arrSpeedSP[3] + arrSpeedSP[4] + arrSpeedSP[5] + arrSpeedSP[6]  + arrSpeedSP[7]  + arrSpeedSP[8] + arrSpeedSP[9] ) / 10;

			lpFoc->Iq_des = pidTask_test( &pidSpeed, (int)sp_speed, pv_speed );

			counter2 = 0;
		}
#endif

		//lpFoc->Iq_des = (ai0 - 2047)<<1;
		//lpFoc->Iq_des = 500;
		//lpFoc->Iq_des = 0;
	}

	///////////////////////////////////////////////////////////////////////////
	Ia = -1.0f * (float)( current_a - current_a_offset );
	Ib = -1.0f * (float)( current_b - current_b_offset );
	///////////////////////////////////////////////////////////////////////////
	angle = readRawUVW();
	mcFocSetAngle( lpFoc, angle );
	mcFocSetCurrent( lpFoc, Ia, Ib );
	///////////////////////////////////////////////////////////////////////////
	mcClark( lpFoc );
	mcPark( lpFoc );
	///////////////////////////////////////////////////////////////////////////
	lpFoc->Vd = pidTask( &lpFoc->pid_d, lpFoc->Id_des, lpFoc->Id );
	lpFoc->Vq = pidTask( &lpFoc->pid_q, lpFoc->Iq_des, lpFoc->Iq );
	///////////////////////////////////////////////////////////////////////////
	mcInvPark( lpFoc );
	mcInvClark( lpFoc );
	///////////////////////////////////////////////////////////////////////////
	//mcFocSVPWM_ST2( lpFoc );
	//mcFocSVPWM0( lpFoc );
	mcFocSVPWM_TI(lpFoc);
	///////////////////////////////////////////////////////////////////////////
	TIM_SetCompare1( TIM1, lpFoc->PWM1 );
	TIM_SetCompare2( TIM1, lpFoc->PWM2 );
	TIM_SetCompare3( TIM1, lpFoc->PWM3 );
	///////////////////////////////////////////////////////////////////////////
	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Id + 2047, lpFoc->Iq + 2047 );
	//DAC_SetDualChannelData( DAC_Align_12b_R, ( lpFoc->angle * 10 ), lpFoc->Iq + 2047 );
	DAC_SetDualChannelData( DAC_Align_12b_R, (int16_t)f_rpm_mt/3*10 + 2047, lpFoc->Iq + 2047 );
//enc_delta
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

	*current_a = ( arrIa[0] + arrIa[1] + arrIa[2] + arrIa[3] + arrIa[4] + arrIa[5] + arrIa[6]  + arrIa[7]  + arrIa[8] + arrIa[9] ) / 10;
	*current_b = ( arrIb[0] + arrIb[1] + arrIb[2] + arrIb[3] + arrIb[4] + arrIb[5] + arrIb[6]  + arrIb[7]  + arrIb[8] + arrIb[9] ) / 10;
}
