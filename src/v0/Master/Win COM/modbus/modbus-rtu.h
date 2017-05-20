#include <commctrl.h>

#include "modbus.h"

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
);

// FC5 - Write Internal Bits or Physical Coils
int mbRTUWriteSingleCoil
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int coilAddress,
	unsigned char coilIsOn
);

// FC15 - Internal Bits or Physical Coils
int mbRTUWriteMultipleCoils
(
	LPMB lpMb,
	unsigned char deviceID
);
///////////////////////////////////////////////////////////////////////////
// FC4 - Physical Input Registers
// FC3 - Read Holding Registers
int mbRTUReadInputOrHoldingRegister
(
	LPMB lpMb,
	unsigned char deviceID,
	unsigned int regAddress,
	unsigned char wordCount,
	unsigned int *regValue
);

// FC6 - Write Single Registers or Physical Output Registers
int mbRTUWriteSingleRegister
(
	LPMB lpMb,
	unsigned char deviceID
);

// FC16 - Internal Registers or Physical Output Registers
int mbRTUWriteMultipleRegisters
(
	LPMB lpMb,
	unsigned char deviceID
);
///////////////////////////////////////////////////////////////////////////
int mbRTUMasterReadResponse
(
	LPMB lpMb,
	char deviceID
);

int mbRTUMasterSend
(
	LPMB lpMb
);

int mbRTUMasterConnect
(
	LPMB lpMb,
	int com
);
///////////////////////////////////////////////////////////////////////////
void serialInitDefDCB
(
	DCB *dcb
);

unsigned char initSerial
(
	unsigned int com,
	LPDCB dcb,
	HANDLE *hComThread,
	LPTHREAD_START_ROUTINE lpComThreadFunc,
	LPVOID lpParam
);
///////////////////////////////////////////////////////////////////////////
unsigned char isValidCRC16(unsigned char *buffer, unsigned int len);
unsigned int CRC16(unsigned char *take_val, unsigned int take_val_len);
///////////////////////////////////////////////////////////////////////////