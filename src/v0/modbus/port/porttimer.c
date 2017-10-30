/*
 * FreeModbus Libary: LPC214X Port
 * Copyright (C) 2007 Tiago Prado Lone <tiago@maxwellbohr.com.br>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

BOOL xMBPortTimersInit( USHORT usTim1Timerout50us )
{
	NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef base_timer;

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM7, ENABLE );

    TIM_TimeBaseStructInit( &base_timer );

    /* 84000 к√ц // 4200 = 20 к√ц ( 50 мкс ) */
    base_timer.TIM_Prescaler = 4200 - 1;
    base_timer.TIM_Period = ( (uint32_t) usTim1Timerout50us ) - 1;
    base_timer.TIM_ClockDivision = 0;
    base_timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM7, &base_timer );

    TIM_ClearITPendingBit( TIM7, TIM_IT_Update );

    /* –азрешаем прерывание по обновлению (в данном случае -  по переполнению) счЄтчика-таймера TIM2.  */
    TIM_ITConfig( TIM7, TIM_IT_Update, ENABLE );

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    TIM_Cmd( TIM7, ENABLE );

    return TRUE;
}

void vMBPortTimersEnable()
{
	TIM_SetCounter( TIM7, 0 );
    TIM_Cmd( TIM7, ENABLE );
}

void vMBPortTimersDisable()
{
	TIM_Cmd( TIM7, DISABLE );
}

void TIM7_IRQHandler( void )
{
	if ( TIM_GetITStatus( TIM7, TIM_IT_Update ) != RESET ) {
		TIM_ClearITPendingBit( TIM7, TIM_IT_Update );
		(void) pxMBPortCBTimerExpired();
	}
}
