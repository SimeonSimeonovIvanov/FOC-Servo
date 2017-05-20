#include "stdafx.h"

#include <stdlib.h>

#include "modbus.h"
#include "modbus-rtu.h"
#include "modbus-tcp.h"

/////////////////////////////////////////////////////////////////////////////////////////
// Bit access:
//
// FC2 - Read Discrete Inputs
int mdReadDiscreteInputs
(
	LPMB lpMb,
	unsigned char deviceID,
	char *input,
	unsigned int start,
	unsigned int end
)
{
	char txBuffer[6] = { deviceID, 2, start>>8, start, end>>8, end };
	unsigned int uiResult = 0;

	lpMb->txBufferLenght = sizeof(txBuffer);
	memcpy( lpMb->txBuffer, txBuffer, sizeof(txBuffer) );

	if( lpMb->mbMasterSend( lpMb ) ) {
		return 1;
	}

	switch( lpMb->ascii_or_rtu_or_tcp ) {
	case 0:
	 break;

	case 1:
		uiResult = mdRTUReadDiscreteInputsOrCoils(lpMb, deviceID, input, start, start + end);
	 break;

	case 2:
		uiResult = mdTCPReadDiscreteInputs(lpMb, deviceID, input, start, start + end);		
	 break;
	}

	if( uiResult ) {
		uiResult += 200;
	}

	return uiResult;
}

// FC1 - Read Internal Bits or Physical Coils
int mbReadCoils
(
	LPMB lpMb,
	unsigned char deviceID,
	char *output,
	unsigned int start,
	unsigned int end
)
{
	char txBuffer[] = { deviceID, 1, start>>8, start, end>>8, end };
	unsigned int uiResult = 0;

	lpMb->txBufferLenght = sizeof(txBuffer);
	memcpy( lpMb->txBuffer, txBuffer, sizeof(txBuffer) );

	if( lpMb->mbMasterSend( lpMb ) ) {
		return 1;
	}

	switch( lpMb->ascii_or_rtu_or_tcp ) {
	case 0:
	 break;

	case 1:
		uiResult = mdRTUReadDiscreteInputsOrCoils( lpMb, deviceID, output, start, end );
	 break;

	case 2:
		uiResult = mbTCPReadCoils( lpMb, deviceID, output, start, end );
	 break;
	}

	if( uiResult ) {
		uiResult += 200;
	}

	return uiResult;
}

// FC5 - Write Internal Bits or Physical Coils
int mbWriteSingleCoil
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int coilAddress,
	unsigned char coilIsOn
)
{
	char txBuffer[] = { (char)deviceID, 5, coilAddress>>8, coilAddress, 0, 0 };
	unsigned int uiResult = 0;

	if(coilIsOn) txBuffer[4] = (char)0xff;

	lpMb->txBufferLenght = sizeof(txBuffer);
	memcpy( lpMb->txBuffer, txBuffer, sizeof(txBuffer) );

	if( lpMb->mbMasterSend( lpMb ) ) {
		return 1;
	}

	switch( lpMb->ascii_or_rtu_or_tcp ) {
	case 0:
	 break;

	case 1:
		uiResult = mbRTUWriteSingleCoil(lpMb, deviceID, coilAddress, coilIsOn);
	 break;

	case 2:
		uiResult = mbTCPWriteSingleCoil(lpMb, deviceID, coilAddress, coilIsOn);
	 break;
	}

	if( uiResult ) {
		uiResult += 200;
	}

	return uiResult;
}

// FC15 - Internal Bits or Physical Coils
int mbWriteMultipleCoils
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int coilAddress,
	unsigned char numberOfCoil,
	unsigned char *arrCoil
) // ???
{
	char txBuffer[] = {
		(char)deviceID, 15,
		coilAddress>>8, coilAddress,
		numberOfCoil>>8, numberOfCoil,
		( numberOfCoil + 7 ) / 8
	};

	unsigned int i, uiResult = 0;

	memset( lpMb->txBuffer, 0, sizeof( lpMb->txBuffer ) );

	lpMb->txBufferLenght = sizeof(txBuffer) + ( ( numberOfCoil + 7 ) / 8 );
	memcpy( lpMb->txBuffer, txBuffer, sizeof(txBuffer) );

	for( i = 0; i < numberOfCoil; i++) {
		lpMb->txBuffer[ 7 + i / 8 ] |= arrCoil[ i ]<<( i - 8 * ( i / 8 ) );
	}

	if( lpMb->mbMasterSend( lpMb ) ) {
		return 1;
	}

	switch( lpMb->ascii_or_rtu_or_tcp ) {
	case 0:
	 break;

	case 1:
		uiResult = mbRTUWriteMultipleCoils( lpMb, deviceID );
	 break;

	case 2: // ???
		uiResult = mbTCPWriteMultipleCoils( lpMb, deviceID );
	 break;
	}

	if( uiResult ) {
		uiResult += 200;
	}

	return uiResult;
}
/////////////////////////////////////////////////////////////////////////////////////////
// 16-bit access:
//
// FC4 - Physical Input Registers
int mbReadInputRegister
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned int wordCount,
	unsigned int *regValue
)
{
	char txBuffer[] = { (char)deviceID, 4, regAddress>>8, regAddress, wordCount>>8, wordCount };
	unsigned int uiResult = 0;

	lpMb->txBufferLenght = sizeof(txBuffer);
	memcpy( lpMb->txBuffer, txBuffer, sizeof(txBuffer) );

	if( lpMb->mbMasterSend( lpMb ) ) {
		return 1;
	}

	switch( lpMb->ascii_or_rtu_or_tcp ) {
	case 0:
	 break;

	case 1:
		uiResult = mbRTUReadInputOrHoldingRegister(lpMb, deviceID, regAddress, wordCount, regValue);
	 break;
	case 2:
		uiResult = mbTCPReadInputRegister(lpMb, deviceID, regAddress, wordCount, regValue);
	 break;
	}

	if( uiResult ) {
		uiResult += 200;
	}

	return uiResult;
}
/////////////////////////////////////////////////////////////////////////////////////////
// FC3 - Read Holding Registers
int mbReadHoldingRegisters
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned int wordCount,
	unsigned int *regValue
)
{
	char txBuffer[] = { (char)deviceID, 3, regAddress>>8, regAddress, wordCount>>8, wordCount };
	unsigned int uiResult = 0;

	lpMb->txBufferLenght = sizeof(txBuffer);
	memcpy( lpMb->txBuffer, txBuffer, sizeof(txBuffer) );

	if( lpMb->mbMasterSend( lpMb ) ) {
		return 1;
	}

	switch( lpMb->ascii_or_rtu_or_tcp ) {
	case 0:
	 break;

	case 1:
		uiResult = mbRTUReadInputOrHoldingRegister( lpMb, deviceID, regAddress, wordCount, regValue );
	 break;

	case 2:
		uiResult = mbTCPReadHoldingRegisters( lpMb, deviceID, regAddress, wordCount, regValue );
	 break;
	}

	if( uiResult ) {
		uiResult += 200;
	}

	return uiResult;
}

// FC6 - Write Single Registers or Physical Output Registers
int mbWriteSingleRegister
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned int regValue
)
{
	char txBuffer[] = { (char)deviceID, 6, regAddress>>8, regAddress, regValue>>8, regValue };
	unsigned int uiResult = 0;

	lpMb->txBufferLenght = sizeof(txBuffer);
	memcpy( lpMb->txBuffer, txBuffer, sizeof(txBuffer) );

	if( lpMb->mbMasterSend( lpMb ) ) {
		return 1;
	}

	switch( lpMb->ascii_or_rtu_or_tcp ) {
	case 0:
	 break;

	case 1:
		uiResult = mbRTUWriteSingleRegister( lpMb, deviceID );
	 break;

	case 2:
		uiResult = mbTCPWriteSingleRegister(lpMb, deviceID, regAddress, regValue);
	 break;
	}

	if( uiResult ) {
		uiResult += 200;
	}

	return uiResult;
}

// FC16 - Internal Registers or Physical Output Registers
int mbWriteMultipleRegisters
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned int wordCount,
	unsigned int *regValue
)
{
	unsigned char txBuffer[] = {
		deviceID, 16, regAddress>>8, regAddress, wordCount>>8, wordCount, 2 * wordCount
	};
	unsigned int i, uiResult = 0;

	lpMb->txBufferLenght = sizeof(txBuffer) + sizeof(char) * 2 * wordCount;
	memcpy( lpMb->txBuffer, txBuffer, sizeof(txBuffer ) );
	
	for( i = 0; i < wordCount; i++ ) {
		lpMb->txBuffer[ 7 + 2 * i ] = regValue[ i ]>>8;
		lpMb->txBuffer[ 8 + 2 * i ] = regValue[ i ];
	}

	if( lpMb->mbMasterSend( lpMb ) ) {
		return 1;
	}

	switch( lpMb->ascii_or_rtu_or_tcp ) {
	case 0:
	 break;

	case 1:
		uiResult = mbRTUWriteMultipleRegisters( lpMb, deviceID );
	 break;

	case 2:
		uiResult = mbTCPWriteMultipleRegisters(lpMb, deviceID, regAddress, wordCount);
	 break;
	}

	if( uiResult ) {
		uiResult += 200;
	}

	return uiResult;
}

// FC23 - Internal Registers or Physical Output Registers
void mbReadWriteMultipleRegisters
(
	LPMB lpMb
)
{
}
/////////////////////////////////////////////////////////////////////////////////////////
int mbMasterInit
(
	LPMB lpMb,
	unsigned char type
)
{
	int (*lpMbTxFunc[3])(LPMB lpMb) = {
		mbMasterSendAscii,
		mbRTUMasterSend,
		mbTCPMasterSend
	};

	if( type > 2 ) {
		return 1;
	}

	if( 1 == type ) {
		//serialInitDefDCB( &lpMb->dcb );
	}

	lpMb->TID = lpMb->enable = 0;
	lpMb->ascii_or_rtu_or_tcp = type;
	lpMb->mbMasterSend = lpMbTxFunc[ lpMb->ascii_or_rtu_or_tcp ];

	return 0;
}

int mbMasterConnect
(
	LPMB lpMb
)
{
	return 0;
}

int mbMasterDisconnect
(
	LPMB lpMb
)
{
	switch( lpMb->ascii_or_rtu_or_tcp ) {
	case 0:
	 break;

	case 1:
		if( INVALID_HANDLE_VALUE != lpMb->hCom ) {
			CloseHandle( lpMb->hCom);
			lpMb->hCom = INVALID_HANDLE_VALUE;
		}
		 break;

	case 2:
		if( lpMb->tcpSocket ) {
			closesocket( lpMb->tcpSocket );
		}
		WSACleanup();
	 break;
	}

	lpMb->enable = 0;

	return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////
int mbMasterSendAscii
(
	LPMB lpMb
)
{
	return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////
unsigned int mbSItoU32( int iValue )
{
	unsigned int uiValue = iValue;

	if( 0 > iValue ) {
		uiValue &= 0x7FFFFFFF;
		uiValue |= 0x80000000;
	}

	return uiValue;
}

int mbU32toSI( unsigned int uiValue )
{
	int iValue = uiValue;

	if( 0x80000000 & iValue ) {
		iValue = ( 0x7FFFFFFF & iValue ) - 0x80000000;
	}

	return iValue;
}
/////////////////////////////////////////////////////////////////////////////////////////
unsigned int mbSItoU16( int iValue )
{
	unsigned int uiValue = iValue;

	if( 0 > iValue ) {
		uiValue &= 0x7FFF;
		uiValue |= 0x8000;
	}

	return uiValue;
}

int mbU16toSI( unsigned int uiValue )
{
	int iValue = uiValue;

	if( 0x8000 & iValue ) {
		iValue = ( 0x7FFF & iValue ) - 32768;
	}

	return iValue;
}
/////////////////////////////////////////////////////////////////////////////////////////