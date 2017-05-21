#include "svpwm.h"

extern volatile uint16_t ADC_values[ARRAYSIZE];

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

	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC4Ref );
	TIM_ARRPreloadConfig( TIM1, ENABLE );

	DBGMCU->APB2FZ |= DBGMCU_APB1_FZ_DBG_TIM1_STOP;

	TIM_Cmd( TIM1, ENABLE );
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
	ADC_CommonInitStruct.ADC_Mode =  ADC_DualMode_RegSimult_InjecSimult;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit( &ADC_CommonInitStruct );
	// ------------------------------------------------------------------------------------------------------------------------
	ADC_StructInit( &ADC_CommonInitStruct );
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = 0;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 2;
	ADC_Init( ADC1, &ADC_InitStructure );
	//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_NbrOfConversion = 0;
	ADC_Init( ADC2, &ADC_InitStructure );
	// ------------------------------------------------------------------------------------------------------------------------
	ADC_RegularChannelConfig( ADC1, AIN1_ADC_CHANNEL, 1, ADC_SampleTime_84Cycles );
	ADC_RegularChannelConfig( ADC1, AIN2_ADC_CHANNEL, 2, ADC_SampleTime_84Cycles );
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	ADC_InjectedSequencerLengthConfig( ADC1, 2 );
	ADC_InjectedSequencerLengthConfig( ADC2, 2 );

	ADC_InjectedChannelConfig( ADC1, PHASE_A_ADC_CHANNEL, 1, SAMPLING_TIME_CK );
	ADC_InjectedChannelConfig( ADC1, VOLT_FDBK_CHANNEL, 2, SAMPLING_TIME_CK );

	ADC_InjectedChannelConfig( ADC2, PHASE_B_ADC_CHANNEL, 1, SAMPLING_TIME_CK );
	ADC_InjectedChannelConfig( ADC2, AIN0_ADC_CHANNEL, 2, SAMPLING_TIME_CK );
	
	ADC_ExternalTrigInjectedConvConfig( ADC1, ADC_ExternalTrigInjecConv_T1_CC4 );
	ADC_ExternalTrigInjectedConvEdgeConfig( ADC1, ADC_ExternalTrigInjecConvEdge_Rising );
	
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
	NVIC_EnableIRQ(ADC_IRQn);

	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd( ADC1, ENABLE );
	ADC_Cmd( ADC2, ENABLE );

	ADC_ITConfig( ADC1, ADC_IT_JEOC | ADC_IT_EOC, ENABLE );

	ADC_SoftwareStartConv(ADC1);
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

// Magnitude must not be larger than sqrt(3)/2, or 0.866
static void svm
(
	float alpha, float beta, uint32_t PWMHalfPeriod,
	uint32_t* tAout, uint32_t* tBout, uint32_t* tCout
)
{
	uint32_t sector;

	if (beta >= 0.0f) {
		if (alpha >= 0.0f) {
			//quadrant I
			if (ONE_BY_SQRT3 * beta > alpha)
				sector = 2;
			else
				sector = 1;
		}
		else {
			//quadrant II
			if (-ONE_BY_SQRT3 * beta > alpha)
				sector = 3;
			else
				sector = 2;
		}
	}
	else {
		if (alpha >= 0.0f) {
			//quadrant IV5
			if (-ONE_BY_SQRT3 * beta > alpha)
				sector = 5;
			else
				sector = 6;
		}
		else {
			//quadrant III
			if (ONE_BY_SQRT3 * beta > alpha)
				sector = 4;
			else
				sector = 5;
		}
	}

	// PWM timings
	uint32_t tA, tB, tC;

	switch (sector) {

		// sector 1-2
	case 1: {
		// Vector on-times
		uint32_t t1 = (alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t2 = (TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tA = (PWMHalfPeriod - t1 - t2) / 2;
		tB = tA + t1;
		tC = tB + t2;

		break;
	}

			// sector 2-3
	case 2: {
		// Vector on-times
		uint32_t t2 = (alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t3 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tB = (PWMHalfPeriod - t2 - t3) / 2;
		tA = tB + t3;
		tC = tA + t2;

		break;
	}

			// sector 3-4
	case 3: {
		// Vector on-times
		uint32_t t3 = (TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t4 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tB = (PWMHalfPeriod - t3 - t4) / 2;
		tC = tB + t3;
		tA = tC + t4;

		break;
	}

			// sector 4-5
	case 4: {
		// Vector on-times
		uint32_t t4 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t5 = (-TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tC = (PWMHalfPeriod - t4 - t5) / 2;
		tB = tC + t5;
		tA = tB + t4;

		break;
	}

			// sector 5-6
	case 5: {
		// Vector on-times
		uint32_t t5 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t6 = (alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tC = (PWMHalfPeriod - t5 - t6) / 2;
		tA = tC + t5;
		tB = tA + t6;

		break;
	}

			// sector 6-1
	case 6: {
		// Vector on-times
		uint32_t t6 = (-TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t1 = (alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tA = (PWMHalfPeriod - t6 - t1) / 2;
		tC = tA + t1;
		tB = tC + t6;

		break;
	}
	}

	*tAout = tA;
	*tBout = tB;
	*tCout = tC;
}

/**
* Truncate the magnitude of a vector.
*
* @param x
* The first component.
*
* @param y
* The second component.
*
* @param max
* The maximum magnitude.
*
* @return
* True if saturation happened, false otherwise
*/
char utils_saturate_vector_2d(float *x, float *y, float max) {
	char retval = 0;
	float mag = sqrtf(*x * *x + *y * *y);
	max = fabsf(max);

	if (mag < 1e-10) {
		mag = 1e-10;
	}

	if (mag > max) {
		const float f = max / mag;
		*x *= f;
		*y *= f;
		retval = 1;
	}

	return retval;
}

void mcFocSVPWM5(LP_MC_FOC lpFoc)
{
	float mod_alpha, mod_beta;
	float mod_alpha_comp = 0;
	float mod_beta_comp = 0;
	float mod_d, mod_q;
	float max_duty;
	float Vd, Vq;
	float v_bus;

	max_duty = 8000;

	Vd = 0; lpFoc->Vd;
	Vq = 4; lpFoc->Vq;
	v_bus = 50;

	mod_d = Vd / ((2.0 / 3.0) * v_bus);
	mod_q = Vq / ((2.0 / 3.0) * v_bus);

	/*utils_saturate_vector_2d
	(
		(float*)&mod_d, (float*)&mod_q,
		SQRT3_BY_2 * max_duty
	);

	utils_saturate_vector_2d
	(
		(float*)&Vd, (float*)&Vq,
		(2.0 / 3.0) * max_duty * SQRT3_BY_2 * v_bus
	);*/

	mod_alpha = lpFoc->fCosAngle * mod_d - lpFoc->fSinAngle * mod_q;
	mod_beta  = lpFoc->fCosAngle * mod_q + lpFoc->fSinAngle * mod_d;

	// Set output (HW Dependent)
	uint32_t duty1, duty2, duty3, top = 10000;
	//top = TIM1->ARR;
	svm(-mod_alpha, -mod_beta, top, &duty1, &duty2, &duty3);
	//TIMER_UPDATE_DUTY(duty1, duty2, duty3);

	lpFoc->PWM1 = duty1 / 100;
	lpFoc->PWM2 = duty2 / 100;
	lpFoc->PWM3 = duty3 / 100;
}

void mcFocSVPWM4(LP_MC_FOC lpFoc)
{
	float Uref1, Uref2, Uref3;
	int A = 0, B = 0, C = 0;
  //int sectorIndex, arrSectorMap[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
	int sectorIndex, arrSectorMap[8] = { 0, 2, 6, 1, 4, 3, 5, 7 };
	float X, Y, Z;
	float T0, Tpwm, T1, T2;
	float Udc, SQRT3_Tpwm, SQRT3_Tpwm_Udc;
	float Ta, Tb, Tc;
	float PWM1, PWM2, PWM3;

	//////////////////////////////////////////////////////////////////
	Uref1 = lpFoc->Vbeta;
	Uref2 = (+(SQRT3 * lpFoc->Valpha) - lpFoc->Vbeta);
	Uref3 = (-(SQRT3 * lpFoc->Valpha) - lpFoc->Vbeta);

	if (Uref1 > 0.0f) {
		A = 1;
	}

	if (Uref2 > 0.0f) {
		B = 1;
	}

	if (Uref3 > 0.0f) {
		C = 1;
	}

	sectorIndex = (C*4) + (B*2) + A;
	lpFoc->sector = arrSectorMap[ sectorIndex ];
	//////////////////////////////////////////////////////////////////
	Tpwm = 100;
	Udc = 0.65;

	SQRT3_Tpwm = SQRT3 * Tpwm;
	SQRT3_Tpwm_Udc = SQRT3_Tpwm / Udc;

	X = SQRT3_Tpwm_Udc * lpFoc->Vb;
	Y = SQRT3_Tpwm_Udc * (+(SQRT3_DIV2 * lpFoc->Valpha) + lpFoc->Vbeta);
	Z = SQRT3_Tpwm_Udc * (-(SQRT3_DIV2 * lpFoc->Valpha) + lpFoc->Vbeta);

	switch (lpFoc->sector) {
	case 0:
	case 7:
	break;

	case 1:
		T1 = Z;
		T2 = Y;
	break;

	case 2:
		T1 = Y;
		T2 = -X;
	break;

	case 3:
		T1 = -Z;
		T2 = X;
	break;

	case 4:
		T1 = -X;
		T2 = Z;
	break;

	case 5:
		T1 = X;
		T2 = -Y;
	break;

	case 6:
		T1 = -Y;
		T2 = -Z;
	break;
	}
	T0 = Tpwm - T1 - T2;

	Ta = (Tpwm - T1 - T2) / 4;
	Tb = Ta + ( T1 * 0.5f );
	Tc = Tb + ( T2 * 0.5f );

	switch (lpFoc->sector) {
	case 0:
	case 7:
	break;

	case 1:
		PWM1 = Tb;
		PWM2 = Ta;
		PWM3 = Tc;
	break;

	case 2:
		PWM1 = Ta;
		PWM2 = Tc;
		PWM3 = Tb;
	break;

	case 3:
		PWM1 = Ta;
		PWM2 = Tb;
		PWM3 = Tc;
	break;

	case 4:
		PWM1 = Tc;
		PWM2 = Tb;
		PWM3 = Ta;
	break;

	case 5:
		PWM1 = Tc;
		PWM2 = Ta;
		PWM3 = Tb;
	break;

	case 6:
		PWM1 = Tb;
		PWM2 = Tc;
		PWM3 = Ta;
	break;
	}

	lpFoc->PWM1 = PWM1/100;
	lpFoc->PWM2 = PWM2/100;
	lpFoc->PWM3 = PWM3/100;
}

void mcFocSVPWM3(LP_MC_FOC lpFoc)
{
	int   sector;
	float X, Y, Z;
	int   t1, t2;
	int   T1, T2, T3;
	int pwma, pwmb, pwmc;

	//Modified Inverse Clarke
	X = lpFoc->Vbeta;
	Y = 0.5 * ( lpFoc->Vbeta + (1.73205 * lpFoc->Valpha) );
	Z = 0.5 * ( lpFoc->Vbeta - (1.73205 * lpFoc->Valpha) );

	//Get Sector
	//sector = ip->theta / 170; //60 degrees expressed in counts
	// Apply Math per PWM Sector - Sector 0

	sector = lpFoc->angle / 60;

	switch (sector) {
	case 6:  //Wrap
	case 0:  //
		t1 = (int)X;
		t2 = (int)-Z;
		break;

	case 1:  //
		t1 = (int)Y;
		t2 = (int)Z;
		break;

	case 2:  //
		t1 = (int)-Y;
		t2 = (int)X;
		break;

	case 3:  //
		t1 = (int)Z;
		t2 = (int)-X;
		break;

	case 4:  //
		t1 = (int)-Z;
		t2 = (int)-Y;
		break;

	case 5:  //
		t1 = (int)-X;
		t2 = (int)Y;
		break;
	}

	//Now Calculate Periods
	T1 = (100 - t1 - t2) / 2;   //Max is Value is 1024, Max Input is 512
	T2 = T1 + t1;
	T3 = T2 + t2;

	//Now setup outputs
	//Apply Math per PWM Sector - Sector 0
	switch (sector)
	{
	case 6:
	case 0:  //
		pwma = T3;
		pwmb = T2;
		pwmc = T1;
		break;

	case 1:  //
		pwma = T2;
		pwmb = T3;
		pwmc = T1;
		break;

	case 2:  //
		pwma = T1;
		pwmb = T3;
		pwmc = T2;
		break;

	case 3:  //
		pwma = T1;
		pwmb = T2;
		pwmc = T3;
		break;

	case 4:  //
		pwma = T2;
		pwmb = T1;
		pwmc = T3;
		break;

	case 5:  //
		pwma = T3;
		pwmb = T1;
		pwmc = T2;
		break;
	}

	//PWM Output Now
	lpFoc->PWM1 = pwma / 1;
	lpFoc->PWM2 = pwmb / 1;
	lpFoc->PWM3 = pwmc / 1;
}

void mcFocSVPWM2(LP_MC_FOC lpFoc)
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
	Y = ( +Ualpha + Ubeta ) * 0.5f;
	Z = ( -Ualpha + Ubeta ) * 0.5f;

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


#define T			( PWM_PERIOD )
#define T_SQRT3     ( T * SQRT3 )
#define div			( 1 )

void mcFocSVPWM1(LP_MC_FOC lpFoc) // +
{
	int bSector;
	float wX, wY, wZ;
	float wUAlpha, wUBeta;
	float hTimePhA, hTimePhB, hTimePhC;

	wUAlpha = +( lpFoc->Valpha * T_SQRT3 );
	wUBeta  = -( lpFoc->Vbeta  * T );

	wX = wUBeta;
	wY = ( wUBeta + wUAlpha ) * 0.5f;
	wZ = ( wUBeta - wUAlpha ) * 0.5f;

	if(wY<0) {
		if(wZ<0) {
			bSector = 5;
		} else {// wZ >= 0
			if (wX <= 0) {
				bSector = 4;
			} else {// wX > 0
				bSector = 3;
			}
		}
	} else { // wY > 0
		if(wZ >= 0) {
			bSector = 2;
		} else { // wZ < 0
			if(wX <= 0) {
				bSector = 6;
			} else { // wX > 0
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
}

void mcFocSVPWM(LP_MC_FOC lpFoc)
{
	float X, Y, Z;
	float taon, tbon, tcon;
	float PWMPRD = 500, t1, t2;
	volatile int PWM1, PWM2, PWM3;
	volatile int A = 0, B = 0 , C = 0;

	if ( lpFoc->Va > 0 ) {
		A = 1;
	}
	if ( lpFoc->Vb > 0 ) {
		B = 1;
	}
	if ( lpFoc->Vc > 0 ) {
		C = 1;
	}

	lpFoc->sector = A + (B << 1) + (C << 2);

	float Vdc = ( 380.0f / 380.0f );
	float Vdcinvt = PWMPRD / Vdc;

	X = (      SQRT3 * Vdcinvt * lpFoc->Vbeta );
	Y = ( SQRT3_DIV2 * Vdcinvt * lpFoc->Vbeta ) + ( 1.5f * Vdcinvt * lpFoc->Valpha );
	Z = ( SQRT3_DIV2 * Vdcinvt * lpFoc->Vbeta ) - ( 1.5f * Vdcinvt * lpFoc->Valpha );

	switch( lpFoc->sector ) {
	case 0:
	 break;

	case 1:
		t1 = Z;
		t2 = Y;
	 break;

	case 2:
		t1 = Y;
		t2 = -X;
	 break;

	case 3:
		t1 = -Z;
		t2 = X;
	 break;

	case 4:
		t1 = -X;
		t2 = Z;
	 break;

	case 5:
		t1 = X;
		t2 = -Y;
	 break;

	case 6:
		t1 = -Y;
		t2 = -Z;
	 break;

	case 7:
	 break;
	}

	if ((t1 + t2) > PWMPRD) {
		t1 = t1 * (PWMPRD / (t1 + t2));
		t2 = t2 * (PWMPRD / (t1 + t2));
	}

	taon = (PWMPRD - t1 - t2 ) * 0.5f;
	tbon = taon + t1;
	tcon = tbon + t2;

	switch (lpFoc->sector) {
	case 0:
	 break;

	case 1:
		PWM1 = tbon;
		PWM2 = taon;
		PWM3 = tcon;
	 break;

	case 2:
		PWM1 = tbon;
		PWM2 = tcon;
		PWM3 = tbon;
	 break;

	case 3:
		PWM1 = taon;
		PWM2 = tbon;
		PWM3 = tcon;
	 break;

	case 4:
		PWM1 = tcon;
		PWM2 = tbon;
		PWM3 = taon;
	 break;

	case 5:
		PWM1 = tcon;
		PWM2 = taon;
		PWM3 = tbon;
	 break;

	case 6:
		PWM1 = tbon;
		PWM2 = tcon;
		PWM3 = taon;
	 break;

	case 7:
	 break;
	}

	lpFoc->PWM1 = PWM1;
	lpFoc->PWM2 = PWM2;
	lpFoc->PWM3 = PWM3;
}
