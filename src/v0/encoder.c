#include "encoder.h"

//#define __TIM8_ENCODER__
#ifndef __TIM8_ENCODER__
	#define __TIM8_STEP_DIR__
#endif

static float arr_sin[ ROTOR_ENCODER_PERIOD + 1 ], arr_cos[ ROTOR_ENCODER_PERIOD + 1 ];

uint32_t uwTIM10PulseLength = 0;
int16_t tim8_overflow = 0;

uint16_t sanyo_uvw = 0;

void initEncoder(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	GPIO_StructInit( &GPIO_InitStructure );
	TIM_ICStructInit( &TIM_ICInitStruct );
	TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM8, ENABLE );

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_Init( GPIOC, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_Init( GPIOE, &GPIO_InitStructure );

	GPIO_PinAFConfig( GPIOA, GPIO_PinSource0, GPIO_AF_TIM2 );
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource1, GPIO_AF_TIM2 );

	GPIO_PinAFConfig( GPIOB, GPIO_PinSource4, GPIO_AF_TIM3 );
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource5, GPIO_AF_TIM3 );

	GPIO_PinAFConfig( GPIOB, GPIO_PinSource6, GPIO_AF_TIM4 );
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource7, GPIO_AF_TIM4 );

	GPIO_PinAFConfig( GPIOC, GPIO_PinSource6, GPIO_AF_TIM8 );
	GPIO_PinAFConfig( GPIOC, GPIO_PinSource7, GPIO_AF_TIM8 );

	TIM_TimeBaseStructure.TIM_Prescaler = 0x00;
	TIM_TimeBaseStructure.TIM_Period = 0xffffffff;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );

	TIM_TimeBaseStructure.TIM_Period = ROTOR_ENCODER_PERIOD;
	TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure );

	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseInit( TIM4, &TIM_TimeBaseStructure );

#ifdef __TIM8_STEP_DIR__
	TIM_TimeBaseStructure.TIM_Prescaler = 0x01;
#endif
	TIM_TimeBaseInit( TIM8, &TIM_TimeBaseStructure );

	TIM_EncoderInterfaceConfig( TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );
	TIM_EncoderInterfaceConfig( TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );
	TIM_EncoderInterfaceConfig( TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );

#ifdef __TIM8_STEP_DIR__
	TIM_EncoderInterfaceConfig( TIM8, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_BothEdge );
#endif

#ifdef __TIM8_ENCODER__
	TIM_EncoderInterfaceConfig( TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );
#endif

	TIM_ICInitStruct.TIM_ICFilter = 0x0F;
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInit( TIM2, &TIM_ICInitStruct );
	TIM_ICInit( TIM3, &TIM_ICInitStruct );
	TIM_ICInit( TIM4, &TIM_ICInitStruct );
	TIM_ICInit( TIM8, &TIM_ICInitStruct );

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInit( TIM2, &TIM_ICInitStruct );
	TIM_ICInit( TIM3, &TIM_ICInitStruct );
	TIM_ICInit( TIM4, &TIM_ICInitStruct );

#ifdef __TIM8_STEP_DIR__
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_IndirectTI;
#endif
	TIM_ICInit( TIM8, &TIM_ICInitStruct );

	/* 32 bits. Abs. Pos Timer2 */
	TIM_Cmd( TIM2, ENABLE );
	/* 16 bits. Rotor Abs. Pos Timer3 */
	TIM_Cmd( TIM3, ENABLE );
	TIM_Cmd( TIM4, ENABLE );
	TIM_Cmd( TIM8, ENABLE );

	TIM2->CNT = 0; // Abs. Pos Timer2 ( 32 bits. )
	TIM3->CNT = 0; // Rotor Abs. Pos ( 16 bits. )
	TIM4->CNT = 0; // Speed counter ( 16 bits. )
	TIM8->CNT = 0; // Step / Dir Interface ( 16 bits. + Soft Ext. to 32 bits. )

	encoderInitZ();

	createSinCosTable();

	initTim10();
	initTim8_IRQ();

	if( 0 && !readHallMap() ) {
		for(volatile uint32_t i = 0; i < 2000000; i++); // Power UP Delay: ?.? ms.
		initSanyoWareSaveEncoder();
	}
}

void encoderInitZ(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SYSCFG, ENABLE );

	GPIO_StructInit( &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init( GPIOD, &GPIO_InitStructure );

	SYSCFG_EXTILineConfig( EXTI_PortSourceGPIOD, EXTI_PinSource15 );

	EXTI_InitStruct.EXTI_Line = EXTI_Line15;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init( &EXTI_InitStruct );

	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStruct );
}

void initHall( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );

	GPIO_StructInit( &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init( GPIOD, &GPIO_InitStructure );
}

int32_t iEncoderGetAbsPos(void)
{
	return (int32_t)TIM2->CNT;
}

///////////////////////////////////////////////////////////////////////////////

uint16_t read360(void)
{
	uint16_t encoder;
	encoder = (float)( ROTOR_ENCODER_PERIOD - TIM3->CNT ) * ( 360.f / (float)ROTOR_ENCODER_PERIOD );
	return encoder;
}

uint16_t read360uvwWithOffset( int16_t offset )
{
	int16_t pos;

	pos = read360uvw();

	pos += offset;

	if( pos > 360 ) {
		pos = pos - 360;
	} else {
		if( pos < 0 ) {
			pos = 360 + pos;
		}
	}

	return pos;
}

uint16_t encoderAddOffset( int16_t angle, int16_t offset )
{
	angle += offset;

	if( angle > 360 ) {
		angle = angle - 360;
	}

	if( angle < 0 ) {
		angle = 360 + angle;
	}

	return angle;
}

///////////////////////////////////////////////////////////////////////////////

uint16_t initSanyoWareSaveEncoder(void) // ---
{
	static uint32_t map1[] = {
			0,
			0,
			60*11.375f + 30*11.375f,
			120*11.375f + 30*11.375f,
			180*11.375f + 30*11.375f,
			240*11.375f + 30*11.375f,
			300*11.375f + 30*11.375f,
			0
	};
	static uint16_t hall_map[8] = { 0, 5, 3, 4, 1, 6, 2, 0 };
	uint16_t ab, z, AU, BV, ZW;
	uint16_t uvw = 0;

	ab = GPIO_ReadInputData( GPIOA );
	z = GPIO_ReadInputData( GPIOD );

	uvw |= ( ab & GPIO_Pin_0 ) ? 1:0;
	uvw |= ( ab & GPIO_Pin_1 ) ? 2:0;
	uvw |= ( z & GPIO_Pin_15 ) ? 4:0;

	sanyo_uvw = hall_map[ uvw ];

	TIM3->CNT = map1[ hall_map[ uvw ] ];

	return 0;
}

uint16_t readSanyoWareSaveEncoder(void) // ---
{
	return TIM3->ARR - TIM3->CNT;
}

void EXTI15_10_IRQHandler(void)
{
	static uint32_t first_run = 1;
	if( RESET != EXTI_GetITStatus( EXTI_Line15 ) ) {
		if( first_run ) {
			//TIM3->CNT = 1364;
			first_run = 0;
		}
    	EXTI_ClearITPendingBit( EXTI_Line15 );
    }
}

///////////////////////////////////////////////////////////////////////////////

// 4096 Импулса за половин оборот (180 мех.градуса) на енкодер = 360 ел.градуса ( 4 полюсен мотор ):
uint16_t readRawEncoderWithUVW(void) // !!!
{
	static uint32_t hall_old = 0;
	static uint32_t map1[] = { 0, 0, 60*11.375f, 120*11.375f, 180*11.375f, 240*11.375f, 300*11.375f, 0 };
	static uint32_t map2[] = { 0, 60*11.375f, 120*11.375f, 180*11.375f, 240*11.375f, 300*11.375f, 360*11.375f, 0 };

	uint32_t hall = 0, encoder = 0, hall_angle = 0;

	hall = readHallMap();

	if( hall != hall_old ) {

		if( hall == 1 && hall_old == 6 ) {
			hall_angle = map1[hall];
		} else
		if( hall == 6 && hall_old == 1 ) {
			hall_angle = map2[hall];
		} else {
			if( hall > hall_old ) {
				hall_angle = map1[hall];
			} else {
				hall_angle = map2[hall];
			}
		}

		TIM3->CNT = hall_angle;

		hall_old = hall;
	}

	encoder = ROTOR_ENCODER_PERIOD - TIM3->CNT;

	if( !hall_old ) {
		encoder = ( (float)ROTOR_ENCODER_PERIOD / 360.0f ) * encoderAddOffset( encoder * ( 360.0f / (float)ROTOR_ENCODER_PERIOD ), 30 );
	} else {
		//encoder = ( (float)ROTOR_ENCODER_PERIOD / 360.0f ) * encoderAddOffset( encoder * ( 360.0f / (float)ROTOR_ENCODER_PERIOD ), 0 );
	}

	return encoder;
}

uint16_t read360uvw(void)
{
	static uint32_t hall_old = 0;
	static uint32_t map1[] = { 0, 0, 60, 120, 180, 240, 300, 0 };
	static uint32_t map2[] = { 0, 60, 120, 180, 240, 300, 360, 0 };

	uint32_t hall = 0, encoder = 0, hall_angle = 0;

	hall = readHallMap();

	if( hall != hall_old ) {

		if( hall == 1 && hall_old == 6 ) {
			hall_angle = map1[hall];
		} else
		if( hall == 6 && hall_old == 1 ) {
			hall_angle = map2[hall];
		} else {
			if( hall > hall_old ) {
				hall_angle = map1[hall];
			} else {
				hall_angle = map2[hall];
			}
		}

		hall_angle = ( (float)ROTOR_ENCODER_PERIOD / 360.0f ) * hall_angle;
		TIM3->CNT = hall_angle;
	}

	// 4000 Импулса за половин оборот (180 мех.градуса) на енкодер = 360 ел.градуса ( 4 полюсен мотор ):
	encoder = TIM3->CNT * ( 360.f / (float)ROTOR_ENCODER_PERIOD );

	if( !hall_old ) {
		encoder = encoderAddOffset( encoder, 30 );
	} else {
		encoder = encoderAddOffset( encoder, 0 );
	}

	hall_old = hall;

	return 360 - encoder;
}

///////////////////////////////////////////////////////////////////////////////

// IN:  1  3  2  6  4  5	( Raw value from Hall sensor )
// OUT: 5  4  3  2  1  6	( hall_map[ Raw Hall ] )
uint16_t readHallMap( void ) // !!!
{
	static uint16_t hall_map[8] = {
			0,
			5, 3, 4, 1, 6, 2,
			0
	};

	return hall_map[ readRawHallInput() ];
}

// 60  deg:
// Return raw value from Hall sensor ( CCW, JOG+ )
// 1  3  2  6  4  5 - Index for hall_map in readHallMap()
uint16_t readRawHallInput( void )
{
	uint16_t uvw, U, V, W;

	uvw = GPIO_ReadInputData( GPIOD );

	U = ( uvw & GPIO_Pin_12 ) ? 1:0;
	V = ( uvw & GPIO_Pin_13 ) ? 2:0;
	W = ( uvw & GPIO_Pin_14 ) ? 4:0;

	return U | V | W;
}

///////////////////////////////////////////////////////////////////////////////

void createSinCosTable(void)
{
	for( int i = 0; i <= ROTOR_ENCODER_PERIOD; i++ ) {
		arr_sin[i] = sinf( foc_deg_to_rad( (float)i * ( 360.0f / (float)ROTOR_ENCODER_PERIOD ) ) );
		arr_cos[i] = cosf( foc_deg_to_rad( (float)i * ( 360.0f / (float)ROTOR_ENCODER_PERIOD ) ) );
	}
}

float fSinAngle(int angle)
{
	return arr_sin[angle];
}

float fCosAngle(int angle)
{
	return arr_cos[angle];
}

///////////////////////////////////////////////////////////////////////////////

void initTim10(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	/* TIM10 clock enable */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM10, ENABLE );

	/* GPIOB clock enable */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );

	/* TIM10 channel 1 pin (PB.8) configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOB, &GPIO_InitStructure );

	/* Connect TIM pins to AF3 */
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource8, GPIO_AF_TIM10 );

	/* Enable the TIM10 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	///////////////////////////////////////////////////////////////////////////
	//TIM_TimeBaseStructInit( &TIM_TimeBaseInitStructure );
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	// Timer counter clock = sysclock / ( TIM_Prescaler + 1 )
	TIM_TimeBaseInitStructure.TIM_Prescaler = ( 84 / 3 ) - 1;
	// Period = ( TIM counter clock / TIM output clock ) - 1
	TIM_TimeBaseInitStructure.TIM_Period = 0xffff;
	TIM_TimeBaseInit( TIM10, &TIM_TimeBaseInitStructure );

	//TIM_ICStructInit( &TIM_ICInitStructure );
	/* TIM10 configuration: Input Capture mode ---------------------
	   The external signal is connected to TIM10 CH1 pin (PB.8)
	   The Both edge is used as active edge,
	   The TIM10 CCR1 is used to compute the frequency value
	------------------------------------------------------------ */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0f;
	TIM_ICInit( TIM10, &TIM_ICInitStructure );

	/* TIM enable counter */
	TIM_Cmd( TIM10, ENABLE );

	/* Enable the CC1 Interrupt Request */
	TIM_ITConfig( TIM10, TIM_IT_CC1, ENABLE );
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	if( SET == TIM_GetITStatus( TIM10, TIM_IT_CC1 ) ) {
		TIM_ClearITPendingBit( TIM10, TIM_IT_CC1 );
		uwTIM10PulseLength = TIM_GetCapture1( TIM10 );
		TIM10->CNT = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////

void initTim8_IRQ(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	TIM_ClearITPendingBit( TIM8, TIM_IT_Update );
	TIM_ITConfig( TIM8, TIM_IT_Update, ENABLE );
}

void TIM8_UP_TIM13_IRQHandler(void)
{
	if( SET == TIM_GetITStatus( TIM8, TIM_IT_Update ) ) {
		TIM_ClearITPendingBit( TIM8, TIM_IT_Update );

		if( !( TIM8->CR1 & TIM_CR1_DIR ) ) {
			tim8_overflow++;
		} else {
			tim8_overflow--;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
