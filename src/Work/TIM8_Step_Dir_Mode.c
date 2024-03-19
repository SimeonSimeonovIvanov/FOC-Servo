/*
 * step_dir.c
 *
 *  Created on: Mar 14, 2024
 *      Author: Simeon
 */

#include "main.h"
#include "step_dir.h"

extern TIM_HandleTypeDef htim8;

static int32_t tim8_overflow;

uint8_t getPulseCounter( int32_t *lpCounter )
{
	if( 0 )
	{	// ERROR !!!
		return 1;
	}

	*lpCounter = ( ( 0x0000ffff * (int32_t)tim8_overflow ) + TIM8->CNT );
	return 0;
}

void resetPulseCounter( void )
{	// Step / Dir Interface ( 16 bits. + Soft Ext. to 32 bits. )
	tim8_overflow = 0;
	TIM8->CNT = 0;
}

void PulseCounter_Cmd( FunctionalState NewState )
{
	if( ENABLE == NewState )
	{
		HAL_TIM_Encoder_Start_IT( &htim8, TIM_CHANNEL_ALL );
		HAL_TIM_Base_Start_IT(&htim8);
	} else {
		HAL_TIM_Encoder_Stop_IT( &htim8, TIM_CHANNEL_ALL );
		HAL_TIM_Base_Stop_IT(&htim8);
	}
}

void TIM8_UP_TIM13_IRQHandler(void)
{
	if( (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE) == SET) &&
		(__HAL_TIM_GET_IT_SOURCE(&htim8, TIM_IT_UPDATE) == SET)
	)
	{
		__HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
		if( TIM8->CR1 & TIM_CR1_DIR )
		{
			tim8_overflow--;
		} else {
			tim8_overflow++;
		}
	}
}
//////////////////////////////////////////////////////
// main.c
static void MX_TIM8_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;//TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_INDIRECTTI;//TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0x0f;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0x0f;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}