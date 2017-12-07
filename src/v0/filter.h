#ifndef __FILTER_H__
#define __FILTER_H__

#include "misc.h"

void FirstOrderLagFilter( float *filtered_value, float raw_sensor_value, float k );
float ffilter(float input, float lpArr[], uint32_t size);

#endif
