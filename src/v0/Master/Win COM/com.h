#include <stdio.h>
#include <stdlib.h>
#include <windowsx.h>

#include "resource.h"

#include "dlg/AboutDlg.h"
#include "dlg/adcConstDlg.h"
#include "dlg/dacConstDlg.h"
#include "dlg/mbTimeOutDlg.h"
#include "dlg/tcpSetingsDlg.h"

#include "modbus/modbus.h"
#include "modbus/modbus-rtu.h"
#include "modbus/modbus-tcp.h"

typedef struct PID_DATA {
	//! Last process value, used to find derivative of process value.
	//int16_t
	WORD wLastProcessValue;
	//! Summation of errors, used for integrate calculations
	//int32_t
	DWORD wSumError;
	//! The Proportional tuning constant, multiplied with SCALING_FACTOR
	//int16_t
	int wP_Factor;
	//! The Integral tuning constant, multiplied with SCALING_FACTOR
	//int16_t
	int wI_Factor;
	//! The Derivative tuning constant, multiplied with SCALING_FACTOR
	//int16_t
	int wD_Factor;
	//! Maximum allowed error, avoid overflow
	//int16_t
	WORD wMaxError;
	//! Maximum allowed sumerror, avoid overflow
	DWORD dwMaxSumError;
} pidData_t;

typedef struct {
	HWND hwnd;
	HINSTANCE hInst;

	MB mbMaster;

	char SlaveID;

	HANDLE hComThread;

	unsigned int uiFlagButton;

	unsigned int uiMbTimeOut;
	HWND hwndMbTimeOutDialog;

	int arrAdcConst[7];
	HWND hwndAdcDialog;
	int flagAdcConstId;
	int flagAdcConst;

	int arrDacNewAccess[2];
	int DAC_Access[2];
	HWND hwndDacDialog;
	int flagDacConst;

	unsigned char flagRunStop;
	unsigned char flagWriteSigleCoil, idWriteSigleCoil;

	unsigned char flagButtonOnClick, idButtonOnClick;



	int flagPidWrite;
	HWND hwndServoControlDialog;

	int scale;
	double kp, ki, kd;

	pidData_t pidData;

	int SP_Position;
	int SET_VELOCITY, SET_ACCEL;
	int SET_VELOCITY2, SET_ACCEL2;
	int SET_MAX_SPEED, MAX_PID_OUT;

} MAIN_DATA, *LP_MAIN_DATA;

DWORD WINAPI comThreadFunc(LPVOID lpParam);

int MODBusReadADCValue1000
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int *adcValue
);
int MODBusReadRawADC
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int *adcValue
);
int MODBusReadADCConst
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int *adcConst
);
int MODBusDacReadConst
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char dac, unsigned int *value
);
int MODBusDacSetAccess
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char dac,
	unsigned int access
);
int MODBusDacGetAccess( LPMB lpMb, unsigned char deviceID, unsigned char dac );
int MODBusSetAdcConst
( 
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char adc, unsigned int new_const
);
int MODBusAdcLoadDefConst( LPMB lpMb, unsigned char deviceID );
int MODBusAdcLoadConstFromEEPROM( LPMB lpMb, unsigned char deviceID );
int MODBusAdcSaveConst2EEPROM( LPMB lpMb, unsigned char deviceID );
int MODBusWriteRawDAC
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char dac,
	unsigned int value
);
int MODBusWriteDAC_1000
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char dac,
	unsigned int value
);
int MODBusSetDacConst
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned char dac,
	unsigned int new_const
);
int MODBusDacLoadDefConst( LPMB lpMb, unsigned char deviceID );
int MODBusDacLoadConstFromEEPROM( LPMB lpMb, unsigned char deviceID );
int MODBusDacSaveConst2EEPROM( LPMB lpMb, unsigned char deviceID );

LP_MAIN_DATA mainDataGet(HWND hwnd);
void mainDataSet(HWND hwnd, LP_MAIN_DATA lpMainData);
LRESULT CALLBACK mainWndFunc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);