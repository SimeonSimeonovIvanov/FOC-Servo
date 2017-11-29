#include "filter.h"

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

/*

size := 4;

( 4 - i.1 )	( 4 - (i.1 + 1) )
( 4 - i.2 )	( 4 - (i.2 + 1) )
( 4 - i.3 )	( 4 - (i.3 + 1) )

( 4 - 1 )	( 4 - 2 )
( 4 - 2 )	( 4 - 3 )
( 4 - 3 )	( 4 - 4 )

( 3 )	( 2 )
( 2 )	( 1 )
( 1 )	( 0 )

arrIa[3] = arrIa[2];
arrIa[2] = arrIa[1];
arrIa[1] = arrIa[0];
arrIa[0] = *current_a;

*/
