#include <winsock.h>

typedef struct structMBMaster MB;
typedef struct structMBMaster *LPMB;

#ifndef __MODBUS_H__
#define __MODBUS_H__

struct structMBMaster {
	HWND hwnd;

	unsigned char rxBuffer[256], rxBufferLenght;
	
	int (*mbMasterSend)(LPMB lpMb);
	unsigned char txBuffer[256], txBufferLenght;

	// struct structMBAscii mbAscii;
	// struct structMBRtu mbRtu;
	int uiComIndex;
	HANDLE hCom;
	DCB dcb;

	// struct structMBTcp mbTcp;
	SOCKET tcpSocket;
	char szIP[16];
	char szTcpPort[13];

	

	unsigned int TID;
	char enable, ascii_or_rtu_or_tcp;
};

#endif

///////////////////////////////////////////////////////////////////////////////////
// FC2 - Discrete Inputs
int mdReadDiscreteInputs
(
	LPMB lpMb,
	unsigned char deviceID,
	char *input,
	unsigned int start,
	unsigned int end
);

// FC1 - Internal Bits or Physical Coils
int mbReadCoils
(
	LPMB lpMb,
	unsigned char deviceID,
	char *output,
	unsigned int start,
	unsigned int end
);

// FC5 - Internal Bits or Physical Coils
int mbWriteSingleCoil
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int coilAddress,
	unsigned char coilIsOn
);

// FC15 - Internal Bits or Physical Coils
int mbWriteMultipleCoils
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int coilAddress,
	unsigned char numberOfCoil,
	unsigned char *arrCoil
);
///////////////////////////////////////////////////////////////////////////////////
// FC4 - Physical Input Registers
int mbReadInputRegister
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned int wordCount,
	unsigned int *regValue
);

// FC3 - Holding Registers
int mbReadHoldingRegisters
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned int wordCount,
	unsigned int *regValue
);

// FC6 - Internal Registers or Physical Output Registers
int mbWriteSingleRegister(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned int regValue
);

// FC16 - Internal Registers or Physical Output Registers
int mbWriteMultipleRegisters
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned int wordCount,
	unsigned int *regValue
);
///////////////////////////////////////////////////////////////////////////////////
int mbMasterInit
(
	LPMB lpMb,
	unsigned char type
);

int mbMasterConnect
(
	LPMB lpMb
);

int mbMasterDisconnect
(
	LPMB lpMb
);
///////////////////////////////////////////////////////////////////////////////////
int mbMasterSendAscii
(
	LPMB lpMb
);
///////////////////////////////////////////////////////////////////////////////////
unsigned int mbSItoU32( int iValue );
int mbU32toSI( unsigned int uiValue );
///////////////////////////////////////////////////////////////////////////////////
unsigned int mbSItoU16( int iValue );
int mbU16toSI( unsigned int uiValue );
///////////////////////////////////////////////////////////////////////////////////