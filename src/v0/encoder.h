#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "misc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

#define ROTOR_ENCODER_PERIOD	( 4095 ) // TIM3->ARR

#define foc_deg_to_rad( deg )	( (float)deg * ( 3.14159265359f / 180.0f ) )

void initEncoder(void);
void encoderInitZ(void);

void initHall(void);

uint16_t read360(void);
uint16_t read360uvw(void);
uint16_t read360uvwWithOffset( int16_t offset );

uint16_t readRawEncoderWithUVW(void);

uint16_t initSanyoWareSaveEncoder(void);
uint16_t readSanyoWareSaveEncoder(void);

uint16_t readHallMap( void );
uint16_t readRawHallInput(void);

float fSinAngle(int angle);
float fCosAngle(int angle);
void createSinCosTable(void);

int32_t iEncoderGetAbsPos(void); // TIM2 ( 32 bits )

#endif
