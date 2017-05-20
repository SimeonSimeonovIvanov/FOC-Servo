#include "stdafx.h"

#include <stdio.h>
#include <stdlib.h>
#include <windows.h>

#include "modbus.h"
#include "modbus-rtu.h"

/////////////////////////////////////////////////////////////////////////////////////////
// Bit access:
//
// FC2 - Read Discrete Inputs
// FC1 - Read Internal Bits or Physical Coils
int mdRTUReadDiscreteInputsOrCoils
(
	LPMB lpMb,
	unsigned char deviceID,
	char *io,
	unsigned int start,
	unsigned int end
)
{
	unsigned int i, uiResult;

	uiResult = mbRTUMasterReadResponse( lpMb, deviceID );

	if( uiResult ) {
		return uiResult;
	}

	if( (6 + (end - start) / 8 ) != lpMb->rxBufferLenght ) {
		return 1;
	}

	for( i = start; i <= end - start; i++ ) {
		io[i] = 1 & ( lpMb->rxBuffer[3 + i/8]>>(i - (8 * ( i / 8 ))) );
	}

	return 0;
}

// FC5 - Write Internal Bits or Physical Coils
int mbRTUWriteSingleCoil
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int coilAddress,
	unsigned char coilIsOn
)
{
	return mbRTUMasterReadResponse( lpMb, deviceID );
}

// FC15 - Internal Bits or Physical Coils
int mbRTUWriteMultipleCoils
(
	LPMB lpMb,
	unsigned char deviceID
)
{
	return mbRTUMasterReadResponse( lpMb, deviceID );
}
//////////////////////////////////////////////////////////////////////////////////////
// 16-bit access:
//
// FC4 - Read Physical Input Registers
// FC3 - Read Holding Registers
int mbRTUReadInputOrHoldingRegister
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned char wordCount,
	unsigned int *regValue
)
{
	unsigned int i, j = 2, uiResult;

	uiResult = mbRTUMasterReadResponse( lpMb, deviceID );

	if( uiResult ) {
		return uiResult;
	}

	if( ( 5 + 2 * wordCount ) != lpMb->rxBufferLenght ) {
		return 1;
	}

	for( i = 0; i < wordCount; i++ ) {
		regValue[i]  = lpMb->rxBuffer[++j]<<8;
		regValue[i] |= lpMb->rxBuffer[++j];
	}

	return 0;
}

// FC6 - Write Internal Registers or Physical Output Registers
int mbRTUWriteSingleRegister
(
	LPMB lpMb,
	unsigned char deviceID
)
{
	unsigned int uiResult;

	uiResult = mbRTUMasterReadResponse( lpMb, deviceID );

	if( uiResult ) {
		return uiResult;
	}

	return 0;
}

// FC16 - Internal Registers or Physical Output Registers
int mbRTUWriteMultipleRegisters
(
	LPMB lpMb,
	unsigned char deviceID
)
{
	unsigned int uiResult;

	uiResult = mbRTUMasterReadResponse( lpMb, deviceID );

	if( uiResult ) {
		return uiResult;
	}

	return 0;
}
//////////////////////////////////////////////////////////////////////////////////////
int mbRTUMasterReadResponse
(
	LPMB lpMb,
	char deviceID
)
{
	OVERLAPPED gOverLapped = { 0 };
	unsigned long dwBytesRead = 100;
	DWORD dwError;
	BOOL bResult;
	int iResult;

	lpMb->rxBufferLenght = 0;

	// set up overlapped structure fields
	gOverLapped.Offset = 0;
	gOverLapped.OffsetHigh = 0;
	gOverLapped.hEvent = NULL;//hEvent; 

	iResult = ReadFile( lpMb->hCom, lpMb->rxBuffer, sizeof(lpMb->rxBuffer), &dwBytesRead, &gOverLapped );

	// If there was a problem, or the async. operation's still pending ...
	if( !iResult ) {
		// Deal with the error code
		switch( dwError = GetLastError() ) {
		case ERROR_HANDLE_EOF: {
			// We're reached the end of the file
			// during the call to ReadFile
			// code to handle that
		}
		 break;

		case ERROR_IO_PENDING: {
			// Asynchronous i/o is still in progress
			// do something else for a while
			// GoDoSomethingElse() ;
			
			// Check on the results of the asynchronous read
			bResult = GetOverlappedResult( lpMb->hCom, &gOverLapped, &dwBytesRead, FALSE );
			// If there was a problem ...
			if( !bResult ) {
				// Deal with the error code
				switch( dwError = GetLastError() ) {
				case ERROR_HANDLE_EOF: {
					// We're reached the end of the file
					//during asynchronous operation
				}
				 break;
				// Deal with other error cases
				}
			}
		} // End case		
		// Deal with other error cases
		} // End switch
	} // End if 

	if( 256 <= dwBytesRead ) {
		return 1;
	}

	lpMb->rxBufferLenght = (unsigned char)dwBytesRead;

	if( TRUE != isValidCRC16( lpMb->rxBuffer, dwBytesRead ) ) {
		return 2;
	}

	if( 0x80 <= lpMb->rxBuffer[1] && 0x90 >= lpMb->rxBuffer[1] ) {
		return (int)lpMb->rxBuffer[2]<<8 | (int)lpMb->rxBuffer[1];
	}

	if( deviceID != lpMb->rxBuffer[0] ) {
		return 3;
	}

	if( lpMb->rxBuffer[1] != lpMb->txBuffer[1] ) {
		return 4;
	}

	return 0;
}

int mbRTUMasterSend( LPMB lpMb )
{
	char *szSend;
	unsigned int crc16;
	unsigned char temp[2];
	unsigned long dwBytesWrite;
	static unsigned char flag = 0;

	if( !lpMb->enable ) {
		return 1;
	}

	if(INVALID_HANDLE_VALUE == lpMb->hCom) {
		return 1;
	}

	//while( flag ) {
		//Sleep(1);
	//}

	flag = 1;

	crc16 = CRC16( lpMb->txBuffer, lpMb->txBufferLenght );
	temp[0] = 0xff & crc16;
	temp[1] = crc16>>8;

	szSend = (char*)malloc( sizeof(char) * (2 + lpMb->txBufferLenght) );

	szSend[ 0 + lpMb->txBufferLenght ] = temp[0];
	szSend[ 1 + lpMb->txBufferLenght ] = temp[1];
	
	memcpy( szSend, lpMb->txBuffer, sizeof(char) * lpMb->txBufferLenght );

	WriteFile(lpMb->hCom, szSend, 2 + lpMb->txBufferLenght, &dwBytesWrite, NULL);

	free(szSend);

	flag = 0;

	if( ( 2 + lpMb->txBufferLenght ) != (unsigned char)dwBytesWrite ) {
		return 2;
	}

	return 0;
}

int mbRTUMasterConnect
(
	LPMB lpMb,
	int com
)
{
	OVERLAPPED gOverLapped = { 0 };
	COMMTIMEOUTS cto;
	char buffer[11];

	// Set up overlapped structure fields:
	//gOverLapped.Offset = 0;
	//gOverLapped.OffsetHigh = 0;
	//gOverLapped.hEvent = NULL;//hEvent;

	if(INVALID_HANDLE_VALUE != lpMb->hCom) {
		CloseHandle(lpMb->hCom);
	}

	sprintf(buffer, "COM%d", com);

	lpMb->hCom = CreateFile(
		buffer,
		GENERIC_READ | GENERIC_WRITE,
		0, // FILE_SHARE_READ | FILE_SHARE_WRITE,
		NULL,
		OPEN_EXISTING,
		0, // FILE_FLAG_OVERLAPPED, FILE_FLAG_NO_BUFFERING,
		NULL // &gOverLapped
	);

	if( INVALID_HANDLE_VALUE == lpMb->hCom ) {
		return 1;
	}

	if( TRUE != GetCommState( lpMb->hCom, &lpMb->dcb ) ) {
		return 2;
	}

	serialInitDefDCB( &lpMb->dcb );

	if( TRUE != SetCommState( lpMb->hCom, &lpMb->dcb ) ) {
		return 3;
	}
	
	if( TRUE != SetCommMask( lpMb->hCom, 0 ) ) {
		return 4;
	}

	cto.ReadIntervalTimeout = 5;
	cto.ReadTotalTimeoutConstant = 5;
	cto.ReadTotalTimeoutMultiplier = 1;
	cto.WriteTotalTimeoutConstant = 0;
	cto.WriteTotalTimeoutMultiplier = 0;

	if( TRUE != SetCommTimeouts( lpMb->hCom, &cto ) ) {
		return 5;
	}

	lpMb->enable = 1;

	return 0;
}
/////////////////////////////////////////////////////////////////////////////////////
void serialInitDefDCB(DCB *dcb)
{
	//memset( dcb, 0, sizeof( DCB ) );

	dcb->DCBlength = sizeof( DCB );
	dcb->BaudRate = CBR_115200; //CBR_19200; //CBR_38400; //CBR_57600; // CBR_115200
	dcb->fBinary = 1;
	//dcb->fParity = 0;
	dcb->fOutxCtsFlow = 0;
	//dcb->fOutxDsrFlow = 0;
	dcb->fDtrControl = 0;
	//dcb->fDsrSensitivity= 0;
	//dcb->fTXContinueOnXoff = 0;
	//dcb->fOutX = 0;
	//dcb->fInX = 0;
	//dcb->fErrorChar = 0;
	//dcb->fNull = 0;
	dcb->fRtsControl = RTS_CONTROL_TOGGLE;
	//dcb->fAbortOnError = 0;
	//dcb->fDummy2 = 0;
	//dcb->wReserved = 0;
	dcb->XonLim = 2048;
	dcb->XoffLim = 512;
	dcb->ByteSize = 8;
	dcb->Parity = EVENPARITY;
	dcb->StopBits = TWOSTOPBITS;
	dcb->XonChar = 19;
	dcb->XoffChar = 25;
	//dcb->ErrorChar = 0;
	//dcb->EofChar = 0;
	//dcb->EvtChar = 0;
	//dcb->wReserved1 = 0;
}

unsigned char isValidCRC16(unsigned char *buffer, unsigned int len)
{	
	if( len > 3 ) {
		unsigned int crc16 = CRC16( buffer, len - 2 );
		if(crc16 == (unsigned)(buffer[len - 2] | (buffer[len - 1]<<8))) return TRUE;
	}
	return FALSE;
}

unsigned int CRC16(unsigned char *take_val, unsigned int take_val_len)
{
	unsigned char i;
	unsigned int i_crc = 0, CRC16 = 0xFFFF;

	while( i_crc < take_val_len ) {
		CRC16 = (0xff00 & CRC16) + (take_val[i_crc]^(0x00ff & CRC16));
		for(i=0; i<8; i++) {
			if( 1 & CRC16 ) CRC16 = (CRC16>>1)^0xA001;
			else CRC16 >>= 1;
		}
		++i_crc;
	}

	return CRC16;
}