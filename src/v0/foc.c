#include "foc.h"

extern int current_a_offset, current_b_offset;
extern int32_t sp_counter;
extern int main_state;

volatile uint16_t current_a, current_b, dc_voltage, ai0, ai1;
volatile uint16_t ADC_values[ARRAYSIZE];
static LP_MC_FOC lpFoc;

PID pidPos;

void focInit(LP_MC_FOC lpFocExt)
{
	lpFoc = lpFocExt;

	memset( lpFoc, 0, sizeof(MC_FOC) );

	focCreateSinCosTable();

	lpFoc->vbus_voltage = 1000.0;

	lpFoc->Id_des = 0;
	lpFoc->Iq_des = 0;

	pidInit( &pidPos, 1.5f, 0.001f, 0.01f, 0.001f );
	pidSetOutLimit( &pidPos, 0.99f, -0.99f );
	pidSetIntegralLimit( &pidPos, 0.99f );
	pidSetInputRange( &pidPos, 450 );

	pidInit( &lpFoc->pid_d, 0.175f, 0.0009f, 0.0f, 0.0009f );
	pidSetOutLimit( &lpFoc->pid_d, 0.9f, -0.9f );
	pidSetIntegralLimit( &lpFoc->pid_d, 0.9f );
	pidSetInputRange( &lpFoc->pid_d, 2040.0f );

	pidInit( &lpFoc->pid_q, 0.175f, 0.0009f, 0.0f, 0.0009f );
	pidSetOutLimit( &lpFoc->pid_q, 0.99f, -0.99f );
	pidSetIntegralLimit( &lpFoc->pid_q, 0.9f );
	pidSetInputRange( &lpFoc->pid_q, 2040.0f );

	initHall();
	initEncoder();
	svpwmInit();
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
	lpFoc->Ibeta = ( lpFoc->Ia + 2 * lpFoc->Ib) * divSQRT3;
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
	static int fFirstRun = 0;
	static int temp = 0;

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
		static int counter = 0;
		//angle = read360();
		angle = read360uvw();

		if( ++counter == 16 ) {
			lpFoc->Iq_des = 2047.0f * pidTask( &pidPos, sp_counter, (float)((int32_t)(TIM2->CNT)) );
			counter = 0;
		}

		//lpFoc->Iq_des = ( ( 4095 - ai0 ) - 2047 );
		//lpFoc->Iq_des = 1500;
	}

	Ia = ( ( 4095 - current_a ) - current_a_offset );
	Ib = ( ( 4095 - current_b ) - current_b_offset );

	mcFocSetAngle( lpFoc, angle );
	mcFocSetCurrent( lpFoc, Ia, Ib );

	mcClark( lpFoc );
	mcPark( lpFoc );

	lpFoc->Vd = pidTask( &lpFoc->pid_d, lpFoc->Id_des, lpFoc->Id );
	lpFoc->Vq = pidTask( &lpFoc->pid_q, lpFoc->Iq_des, lpFoc->Iq );

	mcInvPark( lpFoc );

	lpFoc->Valpha = (float)lpFoc->Valpha;
	lpFoc->Vbeta = (float)lpFoc->Vbeta;

	mcFocSVPWM1( lpFoc );

	TIM_SetCompare1( TIM1, lpFoc->PWM1 );
	TIM_SetCompare2( TIM1, lpFoc->PWM2 );
	TIM_SetCompare3( TIM1, lpFoc->PWM3 );

	GPIO_ResetBits( GPIOB, GPIO_Pin_2 );
}
