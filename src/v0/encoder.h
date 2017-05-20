#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "misc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

#define foc_deg_to_rad( deg ) ( deg * ( 3.14159265359f / 180.0f ) )

void initEncoder(void);
void encoderInitZ(void);

void initHall(void);

uint16_t read360uvw(void);
uint16_t read360uvwWithOffset( int16_t offset );

uint16_t readHallMap( void );
uint16_t readRawHallInput(void);

float fSinAngle(int angle);
float fCosAngle(int angle);
void focCreateSinCosTable(void);

#endif
