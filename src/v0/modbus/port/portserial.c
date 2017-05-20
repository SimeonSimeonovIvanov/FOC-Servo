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
 * File: $Id: portserial.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

BOOL xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	NVIC_InitTypeDef        NVIC_InitStructure;
	GPIO_InitTypeDef        GPIO_InitStructure;
	USART_InitTypeDef       USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;

	/* подавляем предупреждение компилятора о неиспользуемой переменной */
	(void)ucPORT;

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );

	GPIO_PinAFConfig( GPIOD, GPIO_PinSource8, GPIO_AF_USART3 );
	GPIO_PinAFConfig( GPIOD, GPIO_PinSource9, GPIO_AF_USART3 );

	GPIO_StructInit( &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init( GPIOD, &GPIO_InitStructure );

	USART_DeInit( USART3 );

	USART_ClockStructInit( &USART_ClockInitStructure );

	USART_ClockInit( USART3, &USART_ClockInitStructure );

	/* настройка скорости обмена */
	USART_InitStructure.USART_BaudRate = (uint32_t)ulBaudRate;

	/* настройка кол-ва битов данных */
	if( ucDataBits == 9 ) {
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	} else {
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	}

	/* кол-во стоп-битов */
	USART_InitStructure.USART_StopBits = USART_StopBits_2;

	/* настройка паритета (по умолчанию - его нет) */
	switch( eParity ) {
	case MB_PAR_NONE:
		USART_InitStructure.USART_Parity = USART_Parity_No;
	 break;
	case MB_PAR_ODD:
		USART_InitStructure.USART_Parity = USART_Parity_Odd;
	 break;
	case MB_PAR_EVEN:
		USART_InitStructure.USART_Parity = USART_Parity_Even;
	 break;
	default:
		USART_InitStructure.USART_Parity = USART_Parity_No;
	 break;
	};

	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init( USART3, &USART_InitStructure );

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;           /* канал */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   /* приоритет */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          /* приоритет субгруппы */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             /* включаем канал */
	NVIC_Init(&NVIC_InitStructure);                             /* инициализируем */

	USART_Cmd( USART3, ENABLE );

	vMBPortSerialEnable( TRUE, TRUE );

#ifdef RTS_ENABLE
	RTS_INIT;
	RTS_LOW;
#endif
	return TRUE;
}

void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
	if( xRxEnable ) {
		USART_ITConfig( USART3, USART_IT_RXNE, ENABLE );
	} else {
		USART_ITConfig( USART3, USART_IT_RXNE, DISABLE );
	}

	if ( xTxEnable ) {
		USART_ITConfig( USART3, USART_IT_TXE, ENABLE );
		//USART_ITConfig( USART3, USART_IT_TC, ENABLE );
		RTS_HIGH;
	} else {
		USART_ITConfig( USART3, USART_IT_TXE, DISABLE );
		USART_ITConfig( USART3, USART_IT_TC, ENABLE );
	}
}

void USART3_IRQHandler( void )
{
	if ( USART_GetITStatus( USART3, USART_IT_RXNE ) != RESET ) {
		USART_ClearITPendingBit( USART3, USART_IT_RXNE );
		pxMBFrameCBByteReceived();
	}

	if ( USART_GetITStatus( USART3, USART_IT_TC ) != RESET ) {
		USART_ClearITPendingBit( USART3, USART_IT_TC );
		RTS_LOW;
	}

	if ( USART_GetITStatus( USART3, USART_IT_TXE ) != RESET ) {
		USART_ClearITPendingBit( USART3, USART_IT_TXE );
		pxMBFrameCBTransmitterEmpty();
	}
}

BOOL xMBPortSerialPutByte( CHAR ucByte )
{
	USART_SendData( USART3, (uint16_t) ucByte );
	return TRUE;
}

BOOL xMBPortSerialGetByte( CHAR * pucByte )
{
	*pucByte = (CHAR) USART_ReceiveData( USART3 );
	return TRUE;
}
