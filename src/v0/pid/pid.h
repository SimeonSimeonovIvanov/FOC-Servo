#ifndef __PID_H__
#define __PID_H__

typedef struct {
	float kp, ki, kd, dt;

	float error, lastError, dInput, errorScale;
	float sumError, maxSumError;
	float out, maxOut, minOut;
} PID, *LP_PID;

void pidInit( LP_PID lpPid, float kp, float ki, float kd, float dt );
void pidSetOutLimit( LP_PID lpPid, float max, float min );
void pidSetIntegralLimit( LP_PID lpPid, float max );
void pidSetInputRange( LP_PID lpPid, float range );
float pidTask( LP_PID lpPid, float sp, float pv );

void pidInit_test( LP_PID lpPid, float kp, float ki, float kd, float dt );
void pidSetOutLimit_test( LP_PID lpPid, float max, float min );
void pidSetIntegralLimit_test( LP_PID lpPid, float max );
float pidTask_test( LP_PID lpPid, float sp, float pv );

#endif
