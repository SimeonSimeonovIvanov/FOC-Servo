#include "foc.h"

extern int current_a_offset, current_b_offset;
extern int32_t sp_counter;
extern int main_state;

uint16_t current_a, current_b, dc_voltage, ai0, ai1;
uint16_t ADC_values[ARRAYSIZE];
static LP_MC_FOC lpFoc;

PID pidPos;

void focInit(LP_MC_FOC lpFocExt)
{
	DAC_InitTypeDef DAC_InitStruct;

	lpFoc = lpFocExt;

	memset( lpFoc, 0, sizeof( MC_FOC ) );

	lpFoc->vbus_voltage = 40.0f;
	lpFoc->Id_des = 0.0f;
	lpFoc->Iq_des = 0.0f;

	pidInit( &pidPos, 2.0f, 0.001f, 0.00f, 0.001f );
	pidSetOutLimit( &pidPos, 0.99f, -0.99f );
	pidSetIntegralLimit( &pidPos, 0.99f );
	pidSetInputRange( &pidPos, 500 );

	pidInit( &lpFoc->pid_d, 0.4f, 0.0010f, 0.0f, 1.00006f );
	pidSetOutLimit( &lpFoc->pid_d, 0.99f, -0.99f );
	pidSetIntegralLimit( &lpFoc->pid_d, 0.99f );
	pidSetInputRange( &lpFoc->pid_d, 2047.0f );

	pidInit( &lpFoc->pid_q, 0.4f, 0.0010f, 0.0f, 1.00006f );
	pidSetOutLimit( &lpFoc->pid_q, 0.99f, -0.99f );
	pidSetIntegralLimit( &lpFoc->pid_q, 0.99f );
	pidSetInputRange( &lpFoc->pid_q, 2047.0f );

	initHall();
	initEncoder();
	svpwmInit();

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
	lpFoc->Va = ( +( SQRT3 * lpFoc->Valpha ) - lpFoc->Vbeta ) * 0.5f;
	lpFoc->Vc = ( -( SQRT3 * lpFoc->Valpha ) - lpFoc->Vbeta ) * 0.5f;
}

void ADC_IRQHandler( void )
{
	static int arrIa[10]={0}, arrIb[10]={0};

	static int fFirstRun = 0;
	static int temp = 0;

	static float fAngle = 0;
	uint16_t angle;
	float Ia, Ib;

	GPIO_SetBits( GPIOB, GPIO_Pin_2 );

	if( ADC_FLAG_JEOC & ADC1->SR ) {
		current_a = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_1 );
		dc_voltage = ADC_GetInjectedConversionValue( ADC1, ADC_InjectedChannel_2 );

		ADC_ClearITPendingBit( ADC1, ADC_IT_JEOC );
	}

	if( ADC_FLAG_JEOC & ADC2->SR ) {
		current_b = ADC_GetInjectedConversionValue( ADC2, ADC_InjectedChannel_1 );
		ai0 = ADC_GetInjectedConversionValue( ADC2, ADC_InjectedChannel_2 );

		ADC_ClearITPendingBit( ADC2, ADC_IT_JEOC );
	}

	arrIa[9] = arrIa[8];
	arrIa[8] = arrIa[7];
	arrIa[7] = arrIa[6];
	arrIa[6] = arrIa[5];
	arrIa[5] = arrIa[4];
	arrIa[4] = arrIa[3];
	arrIa[3] = arrIa[2];
	arrIa[2] = arrIa[1];
	arrIa[1] = arrIa[0];
	arrIa[0] = current_a;

	arrIb[9] = arrIb[8];
	arrIb[8] = arrIb[7];
	arrIb[7] = arrIb[6];
	arrIb[6] = arrIb[5];
	arrIb[5] = arrIb[4];
	arrIb[4] = arrIb[3];
	arrIb[3] = arrIb[2];
	arrIb[2] = arrIb[1];
	arrIb[1] = arrIb[0];
	arrIb[0] = current_b;

	current_a = ( arrIa[0] + arrIa[1] + arrIa[2] + arrIa[3] + arrIa[4] + arrIa[5] + arrIa[6]  + arrIa[7]  + arrIa[8] + arrIa[9] ) / 10;
	current_b = ( arrIb[0] + arrIb[1] + arrIb[2] + arrIb[3] + arrIb[4] + arrIb[5] + arrIb[6]  + arrIb[7]  + arrIb[8] + arrIb[9] ) / 10;

	if( !main_state ) {
		return;
	}

	if( fFirstRun ) {
		angle = 0;

		lpFoc->Iq_des = temp;

		if( temp < 2000 ) {
			temp += 1;
		} else {
			fFirstRun = 0;
			TIM3->CNT = 0; // for 'angle = read360();'
		}
	}

	if( !fFirstRun ) {
		static int32_t arrPosSP[10], arrPosPV[10], counter = 0;
		int32_t sp_pos, pv_pos;

		//angle = read360();
		angle = readRawUVW();
		//angle = read360uvw();

		sp_pos = sp_counter;
		pv_pos = (int32_t)TIM2->CNT;

		arrPosPV[9] = arrPosPV[8];
		arrPosPV[8] = arrPosPV[7];
		arrPosPV[7] = arrPosPV[6];
		arrPosPV[6] = arrPosPV[5];
		arrPosPV[5] = arrPosPV[4];
		arrPosPV[4] = arrPosPV[3];
		arrPosPV[3] = arrPosPV[2];
		arrPosPV[2] = arrPosPV[1];
		arrPosPV[1] = arrPosPV[0];
		arrPosPV[0] = pv_pos;

		arrPosSP[9] = arrPosSP[8];
		arrPosSP[8] = arrPosSP[7];
		arrPosSP[7] = arrPosSP[6];
		arrPosSP[6] = arrPosSP[5];
		arrPosSP[5] = arrPosSP[4];
		arrPosSP[4] = arrPosSP[3];
		arrPosSP[3] = arrPosSP[2];
		arrPosSP[2] = arrPosSP[1];
		arrPosSP[1] = arrPosSP[0];
		arrPosSP[0] = sp_pos;

		pv_pos = ( arrPosPV[0] + arrPosPV[1] + arrPosPV[2] + arrPosPV[3] + arrPosPV[4] + arrPosPV[5] + arrPosPV[6]  + arrPosPV[7]  + arrPosPV[8] + arrPosPV[9] ) / 10;
		sp_pos = ( arrPosSP[0] + arrPosSP[1] + arrPosSP[2] + arrPosSP[3] + arrPosSP[4] + arrPosSP[5] + arrPosSP[6]  + arrPosSP[7]  + arrPosSP[8] + arrPosSP[9] ) / 10;

		if( ++counter == 16 ) {
			lpFoc->Iq_des = 2047.0f * pidTask( &pidPos, (float)sp_pos, (float)pv_pos );
			counter = 0;
		}

		//lpFoc->Iq_des = ( ( 4095 - ai0 ) - 2047 );
		//lpFoc->Iq_des = 1000;
	}

	Ia = 1.0f * (float)( current_a - current_a_offset );
	Ib = 1.0f * (float)( current_b - current_b_offset );

	mcFocSetAngle( lpFoc, angle );
	mcFocSetCurrent( lpFoc, Ia, Ib );

	mcClark( lpFoc );

	// Вероятно има проблем с подредбата на фазите спрямо Ia = sin( 0 ).
	// В момента Ib = sin(0 + 120) а би трябвало да е Ib = sin( 0 - 120 ) (???):
	lpFoc->Ibeta = -lpFoc->Ibeta;

	mcPark( lpFoc );

	lpFoc->Vd = pidTask( &lpFoc->pid_d, lpFoc->Id_des, lpFoc->Id );
	lpFoc->Vq = pidTask( &lpFoc->pid_q, lpFoc->Iq_des, lpFoc->Iq );

	/*float vfactor = 1.0f / ((2.0f / 3.0f) * lpFoc->vbus_voltage);
	float mod_d = vfactor * lpFoc->Vd;
	float mod_q = vfactor * lpFoc->Vq;
	lpFoc->Vd = mod_d;
	lpFoc->Vq = mod_q;*/

	//lpFoc->Vd = 0.0f;
	//lpFoc->Vq = 0.2f;
	mcInvPark( lpFoc );

	//lpFoc->Valpha = SQRT3_DIV2 * (float)lpFoc->Valpha;
	//lpFoc->Vbeta = SQRT3_DIV2 * (float)lpFoc->Vbeta;
	mcFocSVPWM2( lpFoc );

	TIM_SetCompare1( TIM1, lpFoc->PWM1 );
	TIM_SetCompare2( TIM1, lpFoc->PWM2 );
	TIM_SetCompare3( TIM1, lpFoc->PWM3 );

	//DAC_SetDualChannelData( DAC_Align_12b_R, lpFoc->Id + 2047, lpFoc->Iq + 2047 );
	DAC_SetDualChannelData( DAC_Align_12b_R, ( lpFoc->angle * 10 ), lpFoc->Iq + 2047 );

	GPIO_ResetBits( GPIOB, GPIO_Pin_2 );
}
