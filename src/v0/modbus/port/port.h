/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
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
 * File: $Id: port.h,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */

#ifndef _PORT_H
#define _PORT_H

#define NDEBUG
#define RTS_ENABLE

#include <assert.h>
#include <inttypes.h>

#include "misc.h"

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

#define	INLINE
#define PR_BEGIN_EXTERN_C           extern "C" {
#define	PR_END_EXTERN_C             }

#define ENTER_CRITICAL_SECTION( )	( __disable_irq() )
#define EXIT_CRITICAL_SECTION( )	( __enable_irq() )

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char    CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif


#ifdef RTS_ENABLE

#define RTS_INIT	GPIO_StructInit( &GPIO_InitStructure );				\
					GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;			\
					GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		\
					GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	\
					GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		\
					GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		\
					GPIO_Init( GPIOB, &GPIO_InitStructure );

#define RTS_HIGH	GPIO_SetBits( GPIOB, GPIO_Pin_14 );
#define RTS_LOW		GPIO_ResetBits( GPIOB, GPIO_Pin_14 );

#endif

#endif
