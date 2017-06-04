#include "encoder.h"

//static float arr_sin[361], arr_cos[361];
static float arr_sin[4000], arr_cos[4000];

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

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_Init( GPIOE, &GPIO_InitStructure );

	GPIO_PinAFConfig( GPIOA, GPIO_PinSource0, GPIO_AF_TIM2 );
	GPIO_PinAFConfig( GPIOA, GPIO_PinSource1, GPIO_AF_TIM2 );

	GPIO_PinAFConfig( GPIOB, GPIO_PinSource4, GPIO_AF_TIM3 );
	GPIO_PinAFConfig( GPIOB, GPIO_PinSource5, GPIO_AF_TIM3 );

	TIM_TimeBaseStructure.TIM_Prescaler = 0x00;
	TIM_TimeBaseStructure.TIM_Period = 0xffffffff;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );

	TIM_TimeBaseStructure.TIM_Period = 3999;
	TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure );

	TIM_EncoderInterfaceConfig( TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );
	TIM_EncoderInterfaceConfig( TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising );

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICFilter = 0x0F;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInit( TIM2, &TIM_ICInitStruct );
	TIM_ICInit( TIM3, &TIM_ICInitStruct );

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInit( TIM2, &TIM_ICInitStruct );
	TIM_ICInit( TIM3, &TIM_ICInitStruct );

	TIM_Cmd( TIM2, ENABLE );
	TIM_Cmd( TIM3, ENABLE );

	encoderInitZ();

	TIM2->CNT = 0; // abs.pos
	TIM3->CNT = 0; // angle

	createSinCosTable();
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
	return -( (int32_t)TIM2->CNT );
}

uint16_t read360(void)
{
	uint16_t encoder;
	encoder = TIM3->CNT * 0.09f;
	return 360 - encoder;
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

uint16_t readRawUVW(void)
{
	static uint32_t hall_old = 0;
	static uint32_t map1[] = { 0, 0, 60*11.11f, 120*11.11f, 180*11.11f, 240*11.11f, 300*11.11f, 0 };
	static uint32_t map2[] = { 0, 60*11.11f, 120*11.11f, 180*11.11f, 240*11.11f, 300*11.11f, 360*11.11f, 0 };
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
	}

	// 4000 Импулса за половин оборот (180 мех.градуса) на енкодер = 360 ел.градуса ( 4 полюсен мотор ):
	encoder = TIM3->CNT;

	if( !hall_old ) {
		encoder = 11.11f * encoderAddOffset( encoder * 0.09f, 30 );
	}

	hall_old = hall;

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

		hall_angle = ( 1.0f / 0.09f ) * hall_angle;
		TIM3->CNT = hall_angle;
	}

	// 4000 Импулса за половин оборот (180 мех.градуса) на енкодер = 360 ел.градуса ( 4 полюсен мотор ):
	encoder = TIM3->CNT * 0.09f;

	if( !hall_old ) {
		encoder = encoderAddOffset( encoder, 30 );
	}

	hall_old = hall;

	return 360 - encoder;
}

uint16_t readHallMap( void )
{
	static uint16_t hall_map[8] = {
			0,
			// CW: 1, 3, 2, 6, 4, 5
			1, 3, 2, 5, 6, 4,
			0
	};

	return hall_map[ readRawHallInput() ];
}

uint16_t readRawHallInput( void )
{
	uint16_t uvw, U, V, W;

	uvw = GPIO_ReadInputData( GPIOD );

	U = ( uvw & GPIO_Pin_12 ) ? 4:0;
	V = ( uvw & GPIO_Pin_13 ) ? 2:0;
	W = ( uvw & GPIO_Pin_14 ) ? 1:0;

	return U | V | W;
}

void EXTI15_10_IRQHandler(void)
{
	if( RESET != EXTI_GetITStatus( EXTI_Line15 ) ) {
    	EXTI_ClearITPendingBit( EXTI_Line15 );
    	//TIM3->CNT = 0;
    }
}

void createSinCosTable(void)
{
	for( int i = 0; i < 4000; i++ ) {
		arr_sin[i] = sinf( foc_deg_to_rad( (float)i * 0.09f ) );
		arr_cos[i] = cosf( foc_deg_to_rad( (float)i * 0.09f ) );
	}
	return;

	for( int i = 0; i <= 360; i++) {
		arr_sin[i] = sinf( foc_deg_to_rad( (float)i ) );
		arr_cos[i] = cosf( foc_deg_to_rad( (float)i ) );
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
