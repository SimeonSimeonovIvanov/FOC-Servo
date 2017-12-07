#include "filter.h"

/*
 * http://my.execpc.com/~steidl/robotics/first_order_lag_filter
 *
 * new_filtered_value = k * raw_sensor_value + (1 - k) * old_filtered_value
 *
 * k = 1 - e^ln(1 - Frac) * Ts / Tr
 *
 * Input #1: Maximum Response Time (Tr)
 * This is the (maximum) amount of time (in seconds) between the instant the raw sensor value changes and the instant the filtered sensor value reflects that change.
 *
 * Input #2: Sampling Period (Ts)
 * This is the amount of time (in seconds) between one reading of the raw sensor value and the next.
 *
 * Input #3: Desired Fraction of Change (Frac)
 * This is the (minimum) accuracy with which a change in the raw sensor value (at time t) will be reflected in the filtered sensor value (at or before time t + Tr).
 *
 * 1 - ( e^( ln( 1 - 0.06 ) * ( 0.0006 / 0.02 ) ) ) = 0.00185454032 ( Google calc )
 */
void FirstOrderLagFilter( float *filtered_value, float raw_sensor_value, float k )
{
	*filtered_value = k * raw_sensor_value + ( 1 - k ) * (*filtered_value);
}

/* size := 4;
 *
 * ( 4 - i.1 )	( 4 - (i.1 + 1) )
 * ( 4 - i.2 )	( 4 - (i.2 + 1) )
 * ( 4 - i.3 )	( 4 - (i.3 + 1) )
 *
 * ( 4 - 1 )	( 4 - 2 )
 * ( 4 - 2 )	( 4 - 3 )
 * ( 4 - 3 )	( 4 - 4 )
 *
 * ( 3 )	( 2 )
 * ( 2 )	( 1 )
 * ( 1 )	( 0 )
 *
 * arr[3] = arr[2];
 * arr[2] = arr[1];
 * arr[1] = arr[0];
 * arr[0] = raw;
*/
float ffilter(float input, float lpArr[], uint32_t size)
{
	float sum = 0.0f;
	int i;

	for( i = 1; i < size ; i++ ) {
		lpArr[ size - i ] = lpArr[ size - ( i + 1 ) ];
	}

	lpArr[0] = input;

	for( i = 0; i < size; i++ ) {
		sum += lpArr[ i ];
	}

	sum /= (float)size;
	return sum;
}
