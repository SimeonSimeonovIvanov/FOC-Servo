#include "pid.h"

#include <math.h>

/* Kp - Работи по стандартен начин.
 * Ki - Трябва да се оправи. За момента Kp + Ki = 1.0f. Функцията pidSetIntegralLimit задава максималната 'тежест' на
 * интегралната съставка. Чрез нея и Ki се определя максималната стойност на суматора (цяло число, обикновено >1 ).
 * Това позволява P'I'D-а да работи 'някак'
 */
void pidInit_test( LP_PID lpPid, float kp, float ki, float kd, float dt )
{
	lpPid->kp = kp;
	lpPid->ki = ki * dt;
	lpPid->kd = kd / dt;
	lpPid->dt = dt;

	lpPid->lastError = 0.0f;
	lpPid->sumError = 0.0f;
}

void pidSetOutLimit_test( LP_PID lpPid, float max, float min )
{
	lpPid->maxOut = max;
	lpPid->minOut = min;
}

void pidSetIntegralLimit_test( LP_PID lpPid, float max )
{
	lpPid->maxSumError = max / lpPid->ki;
}

float pidTask_test( LP_PID lpPid, float sp, float pv )
{
	float dInput;

	lpPid->error = ( sp - pv );

	lpPid->sumError += lpPid->error;

	if( lpPid->sumError > lpPid->maxSumError ) {
		lpPid->sumError = lpPid->maxSumError;
	}

	if( lpPid->sumError < -lpPid->maxSumError ) {
		lpPid->sumError = -lpPid->maxSumError;
	}

	dInput = lpPid->error - lpPid->lastError;
	lpPid->lastError = lpPid->error;

	lpPid->out = ( lpPid->kp * lpPid->error ) + ( lpPid->ki * lpPid->sumError ) + ( lpPid->kd * dInput );

	if( lpPid->out > lpPid->maxOut ) {
		lpPid->out = lpPid->maxOut;
	}

	if( lpPid->out < lpPid->minOut ) {
		lpPid->out = lpPid->minOut;
	}

	return lpPid->out;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pidInit( LP_PID lpPid, float kp, float ki, float kd, float dt )
{
	lpPid->kp = kp;
	lpPid->ki = ki;// * dt;
	lpPid->kd = kd;// / dt;
	lpPid->dt = dt;

	lpPid->lastError = 0.0f;
	lpPid->sumError = 0.0f;
}

void pidSetOutLimit( LP_PID lpPid, float max, float min )
{
	lpPid->maxOut = max;
	lpPid->minOut = min;
}

void pidSetIntegralLimit( LP_PID lpPid, float max )
{
	lpPid->maxSumError = max / lpPid->ki;
}

void pidSetInputRange( LP_PID lpPid, float range )
{
	lpPid->errorScale = 1.0f / range;
}

float pidTask( LP_PID lpPid, float sp, float pv )
{
	float error, dInput;
	float sp_in = sp;
	float pv_in = pv;

	error = ( sp_in - pv_in );
	error = error * lpPid->errorScale;

	if( error > 0.999f ) {
		error = 0.999f;
	}

	if( error < -0.999f ) {
		error = -0.999f;
	}

	lpPid->error = error;

	lpPid->sumError += error;

	if( lpPid->sumError > lpPid->maxSumError ) {
		lpPid->sumError = lpPid->maxSumError;
	}

	if( lpPid->sumError < -lpPid->maxSumError ) {
		lpPid->sumError = -lpPid->maxSumError;
	}

	dInput = error - lpPid->lastError;
	lpPid->lastError = error;

	lpPid->out = ( lpPid->kp * error ) + ( lpPid->ki * lpPid->sumError ) + ( lpPid->kd * dInput );

	if( lpPid->out > lpPid->maxOut ) {
		lpPid->out = lpPid->maxOut;
	}

	if( lpPid->out < lpPid->minOut ) {
		lpPid->out = lpPid->minOut;
	}

	return lpPid->out;
}
