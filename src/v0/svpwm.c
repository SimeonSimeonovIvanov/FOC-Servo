#include "svpwm.h"

extern volatile uint16_t ADC_values[ARRAYSIZE];

static float svpwm_sin_table[4096];

void svpwmInit(void)
{
	svpwmInitTIM();
	svpwmInitADC();
}

void svpwmInitTIM( void )
{
	TIM_TimeBaseInitTypeDef	TIM1_TimeBaseStructure;
	TIM_BDTRInitTypeDef		TIM1_BDTRInitStructure;
	TIM_OCInitTypeDef		TIM1_OCInitStructure;
	//NVIC_InitTypeDef		NVIC_InitStructure;

	svpwmInitGPIO();

	svpwmInitSinTable();

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );

	TIM_DeInit( TIM1 );

	TIM_TimeBaseStructInit( &TIM1_TimeBaseStructure );
	TIM1_TimeBaseStructure.TIM_Period = PWM_PERIOD;
	TIM1_TimeBaseStructure.TIM_Prescaler = PWM_PRSC + 1;
	TIM1_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM1_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;
	TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseInit( TIM1, &TIM1_TimeBaseStructure );

	TIM_OCStructInit( &TIM1_OCInitStructure );

	/* Channel 1, 2, 3 and 4 Configuration in PWM mode */
	TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCPolarity_Low;

	TIM1_OCInitStructure.TIM_Pulse = PWM_PERIOD>>1;
	TIM_OC1Init( TIM1, &TIM1_OCInitStructure );

	TIM1_OCInitStructure.TIM_Pulse = PWM_PERIOD>>1;
	TIM_OC2Init( TIM1, &TIM1_OCInitStructure );

	TIM1_OCInitStructure.TIM_Pulse = PWM_PERIOD>>1;
	TIM_OC3Init( TIM1, &TIM1_OCInitStructure );

	TIM1_OCInitStructure.TIM_Pulse = 10;
	TIM_OC4Init( TIM1, &TIM1_OCInitStructure );

	TIM_OC1PreloadConfig( TIM1, TIM_OCPreload_Enable );
	TIM_OC2PreloadConfig( TIM1, TIM_OCPreload_Enable );
	TIM_OC3PreloadConfig( TIM1, TIM_OCPreload_Enable );
	TIM_OC4PreloadConfig( TIM1, TIM_OCPreload_Enable );

	/* Automatic Output enable, Break, dead time and lock configuration*/
	TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM1_BDTRInitStructure.TIM_DeadTime = DEADTIME;
	TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
	TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig( TIM1, &TIM1_BDTRInitStructure );

	TIM_ClearITPendingBit( TIM1, TIM_IT_Break );
	TIM_ITConfig( TIM1, TIM_IT_Break, DISABLE );

	TIM_ARRPreloadConfig( TIM1, ENABLE );

	DBGMCU->APB2FZ |= DBGMCU_APB1_FZ_DBG_TIM1_STOP;

	TIM_SelectOutputTrigger( TIM1, TIM_TRGOSource_Update );

	TIM_Cmd( TIM1, ENABLE );
}

uint16_t svpwmGetFrq( void )
{
	return PWM_FREQ;
}

void svpwmInitGPIO( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE );

	GPIO_StructInit( &GPIO_InitStructure );
	/* GPIOE Configuration: Channel 1, 1N, 2, 2N, 3 and 3N Output */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_9 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init( GPIOE, &GPIO_InitStructure );

	GPIO_PinAFConfig( GPIOE, GPIO_PinSource8, GPIO_AF_TIM1 );
	GPIO_PinAFConfig( GPIOE, GPIO_PinSource9, GPIO_AF_TIM1 );
	GPIO_PinAFConfig( GPIOE, GPIO_PinSource10, GPIO_AF_TIM1 );
	GPIO_PinAFConfig( GPIOE, GPIO_PinSource11, GPIO_AF_TIM1 );
	GPIO_PinAFConfig( GPIOE, GPIO_PinSource12, GPIO_AF_TIM1 );
	GPIO_PinAFConfig( GPIOE, GPIO_PinSource13, GPIO_AF_TIM1 );
	GPIO_PinAFConfig( GPIOE, GPIO_PinSource14, GPIO_AF_TIM1 );

	/* GPIOE Configuration: BKIN pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init( GPIOE, &GPIO_InitStructure );
}

void svpwmInitADC( void )
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_Configuration_Adc1();
	RCC_Configuration_Adc2();
	GPIO_Configuration_Adc1();
	GPIO_Configuration_Adc2();

	DMAInit();

	// ========================================================================================================================
	ADC_CommonStructInit( &ADC_CommonInitStruct );

#ifdef FOC_ADC_Mode_Independent
	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
#endif

#ifdef FOC_ADC_DualMode_RegSimult_InjecSimult
	ADC_CommonInitStruct.ADC_Mode = ADC_DualMode_RegSimult_InjecSimult;
#endif

	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit( &ADC_CommonInitStruct );
	// ------------------------------------------------------------------------------------------------------------------------
	ADC_StructInit( &ADC_InitStructure );
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 2;
	ADC_Init( ADC1, &ADC_InitStructure );
	//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_NbrOfConversion = 0;

#ifdef FOC_ADC_Mode_Independent
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1; // For Resolver interface ????
#endif
	ADC_Init( ADC2, &ADC_InitStructure );
	// ------------------------------------------------------------------------------------------------------------------------
	ADC_RegularChannelConfig( ADC1, AIN1_ADC_CHANNEL, 1, ADC_SampleTime_84Cycles );
	ADC_RegularChannelConfig( ADC1, AIN2_ADC_CHANNEL, 2, ADC_SampleTime_84Cycles );
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	ADC_InjectedSequencerLengthConfig( ADC1, 2 );

	ADC_ExternalTrigInjectedConvConfig( ADC1, ADC_ExternalTrigInjecConv_T1_TRGO );
	//ADC_ExternalTrigInjectedConvConfig( ADC2, ADC_ExternalTrigInjecConv_T1_TRGO );

	ADC_ExternalTrigInjectedConvEdgeConfig( ADC1, ADC_ExternalTrigInjecConvEdge_Rising );
	//ADC_ExternalTrigInjectedConvEdgeConfig( ADC2, ADC_ExternalTrigInjecConvEdge_Rising );

#ifdef FOC_ADC_Mode_Independent
	/*
	 * TIM1(CC4) -> ADC1
	 * TIM8(CC1) -> ADC2***
	 */

	ADC_InjectedSequencerLengthConfig( ADC2, 2 ); // 4 ???

	ADC_InjectedChannelConfig( ADC1, PHASE_A_ADC_CHANNEL, 1, SAMPLING_TIME_CK );
	ADC_InjectedChannelConfig( ADC1, PHASE_B_ADC_CHANNEL, 2, SAMPLING_TIME_CK );

	ADC_InjectedChannelConfig( ADC2, VOLT_FDBK_CHANNEL, 1, SAMPLING_TIME_CK );
	ADC_InjectedChannelConfig( ADC2, AIN0_ADC_CHANNEL, 2, SAMPLING_TIME_CK );

	//ADC_InjectedChannelConfig( ADC2, RESOLVER_SIN_CHANNEL, 3, SAMPLING_TIME_CK );
	//ADC_InjectedChannelConfig( ADC2, RESOLVER_COS_CHANNEL, 4, SAMPLING_TIME_CK );

	ADC_ExternalTrigInjectedConvConfig( ADC2, ADC_ExternalTrigInjecConv_T8_CC2 );
#endif

#ifdef FOC_ADC_DualMode_RegSimult_InjecSimult
	/*
	 * TIM1(CC4) -> ADC1 and ADC2
	 */

	ADC_InjectedSequencerLengthConfig( ADC2, 2 );

	ADC_InjectedChannelConfig( ADC1, PHASE_A_ADC_CHANNEL, 1, SAMPLING_TIME_CK );
	ADC_InjectedChannelConfig( ADC1, VOLT_FDBK_CHANNEL, 2, SAMPLING_TIME_CK );

	ADC_InjectedChannelConfig( ADC2, PHASE_B_ADC_CHANNEL, 1, SAMPLING_TIME_CK );
	ADC_InjectedChannelConfig( ADC2, AIN0_ADC_CHANNEL, 2, SAMPLING_TIME_CK );
#endif

	ADC_AutoInjectedConvCmd( ADC1, DISABLE );
	ADC_AutoInjectedConvCmd( ADC2, DISABLE );

	ADC_InjectedDiscModeCmd( ADC1, DISABLE );
	ADC_InjectedDiscModeCmd( ADC2, DISABLE );
	// ========================================================================================================================

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
	NVIC_EnableIRQ( ADC_IRQn );

	ADC_DMACmd( ADC1, ENABLE );
	ADC_Cmd( ADC1, ENABLE );
	ADC_Cmd( ADC2, ENABLE );

	ADC_ITConfig( ADC1, ADC_IT_JEOC, ENABLE );

	//ADC_SoftwareStartConv( ADC1 );
}

void RCC_Configuration_Adc1( void )
{
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
}

void RCC_Configuration_Adc2( void )
{
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC2, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
}

void GPIO_Configuration_Adc1( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit( &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_1;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

	// PWM Period Debug:
	GPIO_StructInit( &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
}

void GPIO_Configuration_Adc2( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit( &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init( GPIOC, &GPIO_InitStructure );
}

void DMAInit(void){
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_values;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);
}

void DMA1_Stream1_IRQHandler( void )
{

}

/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : STM32x_svpwm_ics.c
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : ICS current reading and PWM generation module
* SVPWM_IcsCalcDutyCycles
********************************************************************************/
void mcFocSVPWM_ST2_TTHI(LP_MC_FOC lpFoc) // +++
{
	int hTimePhA, hTimePhB, hTimePhC;
	int Ualpha, Ubeta;
	int T, T_2, T_4;
	int X, Y, Z;
	int sector;

	T = PWM_PERIOD;
	T_2 = T * 0.50f;
	T_4 = T * 0.25f;

	Ualpha	= T * +lpFoc->Valpha * SQRT3;
	Ubeta	= T * -lpFoc->Vbeta;

	X = Ubeta;
	Y = ( Ubeta - Ualpha ) * 0.5f;
	Z = ( Ubeta + Ualpha ) * 0.5f;

	/* From STM32x_svpwm_ics.c
	 *X = Ubeta;
	 *Y = ( +Ualpha + Ubeta ) * 0.5f;
	 *Z = ( -Ualpha + Ubeta ) * 0.5f;
	 */

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
		hTimePhA = T_4 + ( ( T_2 + X - Z ) * 0.5f );
		hTimePhB = hTimePhA + Z;
		hTimePhC = hTimePhB - X;
	 break;

	case 2:
	case 5:
		hTimePhA = T_4 + ( ( T_2 + Y - Z ) * 0.5f );
		hTimePhB = hTimePhA + Z;
		hTimePhC = hTimePhA - Y;
	 break;

	case 3:
	case 6:
		hTimePhA = T_4 + ( ( T_2 + Y - X ) * 0.5f );
		hTimePhC = hTimePhA - Y;
		hTimePhB = hTimePhC + X;
	 break;
	}

	lpFoc->PWM1 = hTimePhA;
	lpFoc->PWM2 = hTimePhB;
	lpFoc->PWM3 = hTimePhC;
}

void mcFocSVPWM0_TTHI(LP_MC_FOC lpFoc) // +++ ?
{
	float Uref1, Uref2, Uref3;
	float X, Y, Z;
	float t_1, t_2;
	float t1, t2, t3;
	float PWM1, PWM2, PWM3;
	float Tpwm = 1.0;

	Uref1 = lpFoc->Va;
	Uref2 = lpFoc->Vb;
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

	X = ( lpFoc->Vbeta );
	Y = ( lpFoc->Vbeta - ( SQRT3 * lpFoc->Valpha ) ) * 0.5f;
	Z = ( lpFoc->Vbeta + ( SQRT3 * lpFoc->Valpha ) ) * 0.5f;

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

	lpFoc->PWM1 = PWM1 * (float)PWM_PERIOD;
	lpFoc->PWM2 = PWM3 * (float)PWM_PERIOD;
	lpFoc->PWM3 = PWM2 * (float)PWM_PERIOD;
}

void SPWM( LP_MC_FOC lpFoc, float A, uint16_t angle )
{
	float Tpwm = (float)PWM_PERIOD * 0.5f;

	const float k = 360.0f / 4096.0f;

	A=0.5f;
	volatile float u = sinf( foc_deg_to_rad( ((float)angle * k) + 0 ) );
	volatile float v = A * sinf( foc_deg_to_rad( ((float)angle * k) + 120) );
	volatile float w = A * sinf( foc_deg_to_rad( ((float)angle * k) + 240) );

	DAC_SetDualChannelData( DAC_Align_12b_R,  u*1000+1000, lpFoc->PWM2/4 );

	lpFoc->PWM1 = Tpwm + ( u * Tpwm );
	lpFoc->PWM2 = Tpwm + ( v * Tpwm );
	lpFoc->PWM3 = Tpwm + ( w * Tpwm );
}

void mcFocSPWM(LP_MC_FOC lpFoc) // +++
{
	int Tpwm = (float)PWM_PERIOD * 0.5f;

	lpFoc->PWM1 = Tpwm + ( lpFoc->Va * Tpwm );
	lpFoc->PWM2 = Tpwm + ( lpFoc->Vb * Tpwm );
	lpFoc->PWM3 = Tpwm + ( lpFoc->Vc * Tpwm );
}

/*
 *	Triangular Third Harmonic Injection ( TTHI )
 *	http://www.cnblogs.com/nixianmin/p/4791428.html
 *	http://www.e-driveonline.com/main/wp-content/uploads/2015/03/Techincal_paper_Motors_and_drives_Systems_2015_Suda_and_David-2.pdf
 */
void mcFocSVPWM_TTHI(LP_MC_FOC lpFoc) // +++
{
	float Tpwm = (float)PWM_PERIOD * 0.5f;
	volatile float temp, vmin, vmax, Vcom, X, Y, Z;

	if( lpFoc->Va > lpFoc->Vb ) {
		vmax = lpFoc->Va;
		vmin = lpFoc->Vb;
	} else {
		vmax = lpFoc->Vb;
		vmin = lpFoc->Va;
	}

	if( lpFoc->Vc > vmax ) {
		vmax = lpFoc->Vc;
	} else {
		if( lpFoc->Vc < vmin ) {
			vmin = lpFoc->Vc;
		}
	}

	/*
	 * Techincal_paper_Motors_and_drives_Systems_2015_Suda_and_David-2.pdf
	 *
	 * "...0.866, which is the peak of resultant
	 * waveform with third harmonic. This gives us a
	 * modulation factor of 1/0.866, i.e, 1.1547, giving 15.47%
	 * more DC bus utilization."
	 *
	 */
	Vcom = ( vmax + vmin ) * -0.5f;
	X = ( lpFoc->Va + Vcom ) * 1.1547f;
	Y = ( lpFoc->Vb + Vcom ) * 1.1547f;
	Z = ( lpFoc->Vc + Vcom ) * 1.1547f;

	//////////////////////////////////////////////////////////////////////////////
	/*
	X = fLimitValue( X, 0.999 );
	Y = fLimitValue( Y, 0.999 );
	Z = fLimitValue( Z, 0.999 );
	*/
	//////////////////////////////////////////////////////////////////////////////

	lpFoc->PWM1 = Tpwm - ( X * Tpwm );
	lpFoc->PWM2 = Tpwm - ( Y * Tpwm );
	lpFoc->PWM3 = Tpwm - ( Z * Tpwm );
}

/*
 *	Sinusoidal Third Harmonic Injection ( STHI )
 */
void mcFocSVPWM_STHI(LP_MC_FOC lpFoc) // --- ???
{
	float Tpwm = (float)PWM_PERIOD * 0.5f;
	float V, Vcom, X, Y, Z;

	// "sinf(foc_deg_to_rad(lpFoc->angle * 3) - foc_deg_to_rad(90));" -> !!! foc_deg_to_rad(90) = ? if lpFoc.Vd != 0 !!!
	V = ( 1.0f / 6.0f ) * sqrtf( lpFoc->Vq * lpFoc->Vq + lpFoc->Vd * lpFoc->Vd );
	//Vcom = V * sinf( foc_deg_to_rad( lpFoc->angle * 3.0f ) - foc_deg_to_rad( 90 ) );
	Vcom = V * svpwm_sin_table[(int)lpFoc->angle];

	X = ( lpFoc->Va + Vcom ); // * 1.1547f;
	Y = ( lpFoc->Vb + Vcom ); // * 1.1547f;
	Z = ( lpFoc->Vc + Vcom ); // * 1.1547f;

	lpFoc->PWM1 = Tpwm - ( X * Tpwm );
	lpFoc->PWM2 = Tpwm - ( Y * Tpwm );
	lpFoc->PWM3 = Tpwm - ( Z * Tpwm );
}

void svpwmInitSinTable(void)
{
	for( int angle = 0; angle < 4096; angle++ ) {
		svpwm_sin_table[angle] = sinf( foc_deg_to_rad( 3.0f * ((float)angle * (360.0f/4096.0f)) ) - foc_deg_to_rad(90.0f) );
	}
}
