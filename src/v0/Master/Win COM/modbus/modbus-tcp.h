#ifndef __MODBUS_TCP_H__
#define __MODBUS_TCP_H__

#include "modbus.h"

//#include <winsock.h>
#pragma comment(lib, "Ws2_32.lib")

//#include <Mswsock.h>
//#pragma comment(lib, "Mswsock.lib")

// FC2 - Read Discrete Inputs
int mdTCPReadDiscreteInputs
(
	LPMB lpMb,
	unsigned char deviceID,
	char *input, unsigned int start,
	unsigned int end
);

// FC1 - Read Internal Bits or Physical Coils
int mbTCPReadCoils
(
	LPMB lpMb,
	unsigned char deviceID,
	char *output,
	unsigned int start,
	unsigned int lenght
);

// FC5 - Write Internal Bits or Physical Coils
int mbTCPWriteSingleCoil
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int coilAddress,
	unsigned char coilIsOn
);


// FC15 - Internal Bits or Physical Coils
int mbTCPWriteMultipleCoils
(
	LPMB lpMb,
	unsigned char deviceID
);
//////////////////////////////////////////////////////////////////////////////////////
// 16-bit access:
//
// FC4 - Physical Input Registers
int mbTCPReadInputRegister(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned char wordCount,
	unsigned int *regValue
);
// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
// FC3 - Read Holding Registers
int mbTCPReadHoldingRegisters(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned int wordCount,
	unsigned int *regValue
);

// FC6 - Write Internal Registers or Physical Output Registers
int mbTCPWriteSingleRegister(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned int regValue
);

// FC16 - Internal Registers or Physical Output Registers
int mbTCPWriteMultipleRegisters
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned int wordCount
);
//////////////////////////////////////////////////////////////////////////////////////
int mbTCPMasterConnect(
	LPMB lpMb,
	char *szIP,
	char *port
);

int mbTCPMasterSend(
	LPMB lpMb
);

int mbTCPMasterCheckMBAPHeader(
	LPMB lpMb,
	char deviceID,
	char *rxBuffer
);

int mbTCPMsterRead(
	LPMB lpMb,
	char deviceID,
	char *rxBuffer,
	unsigned int len
);

int socketTryConnect
(
	SOCKET *s,
	long hostname,
	int PortNo
);

#endif