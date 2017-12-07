#ifndef __MAIN_H__
#define __MAIN_H__

#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"

#include "mb.h"
#include "mbport.h"

#include "svpwm.h"
#include "encoder.h"

void boardInit(void);
void Configure_PC6(void);

#endif
