#include "main.h"

#define REG_HOLDING_START   40001
#define REG_HOLDING_NREGS   11

void boardInit(void);

extern volatile uint16_t current_a, current_b, dc_voltage, ai0, ai1;
extern PID pidPos;

static USHORT usRegHoldingStart = REG_HOLDING_START;
static USHORT usRegHoldingBuf[ REG_HOLDING_NREGS ];

int current_a_offset = 2047;
int current_b_offset = 2047;

int32_t sp_counter = 0;
int main_state = 0;

MC_FOC stFoc;

int main(void)
{
	int hall;
	int encoder;
	eMBErrorCode eStatus;

	SystemInit();
	SystemCoreClockUpdate();

	boardInit();

	focInit( &stFoc );

	eStatus = eMBInit( MB_RTU, 1, 0, 115200, MB_PAR_NONE );
	if( MB_ENOERR == eStatus ) {
		eStatus = eMBEnable();
	}

	usRegHoldingBuf[9] = 0;

	uint32_t sum_a = 0, sum_b = 0;
	for( int i = 0; i < 100; i++ ) {
		for( volatile uint32_t i = 0; i < 100000; i++ );
		sum_a += current_a;
		sum_b += current_b;
	}

	current_a_offset = sum_a / 100;
	current_b_offset = sum_b / 100;

	main_state = 1;

	while( 1 ) {
		GPIO_ToggleBits( GPIOA, GPIO_Pin_15 );

		hall = readHallMap();
		encoder = read360uvwWithOffset( (int16_t)usRegHoldingBuf[9] );

		//float dc_current = sqrtf( stFoc.Id * stFoc.Id + stFoc.Iq * stFoc.Iq );
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		usRegHoldingBuf[0] = (int)stFoc.Id;
		usRegHoldingBuf[1] = (int)stFoc.Iq;
		//usRegHoldingBuf[0] = -( ( 4095 - current_a ) - current_a_offset );
		//usRegHoldingBuf[1] = -( ( 4095 - current_b ) - current_b_offset );
		//usRegHoldingBuf[1] = 1000 * pidPos.sumError;-( ( 4095 - current_b ) - current_b_offset );
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		usRegHoldingBuf[2] = sp_counter - TIM2->CNT;dc_voltage;
		usRegHoldingBuf[3] = ai0;
		// Encoder 0 ( rot.angle )
		usRegHoldingBuf[4] = hall;
		usRegHoldingBuf[5] = encoder;
		usRegHoldingBuf[6] = TIM3->CNT;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		// Encoder 1 ( abs.pos )
		usRegHoldingBuf[7] = TIM2->CNT;
		usRegHoldingBuf[8] = TIM2->CNT>>16;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		(void)eMBPoll();
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		static int clk_old = 0;
		static int32_t counter = 0;

		uint16_t pinb = GPIO_ReadInputData( GPIOB );
		uint16_t pinc = GPIO_ReadInputData( GPIOC );
		uint8_t clk = ( pinc & GPIO_Pin_6 ) ? 1 : 0;
		uint8_t dir = ( pinb & GPIO_Pin_13 ) ? 1 : 0;

		if( clk && clk != clk_old ) {
			if( dir ) {
				--counter;
			} else {
				++counter;
			}

			sp_counter = counter * 10;
		}

		clk_old = clk;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	}
}

void boardInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	///////////////////////////////////////////////////////////////////////////
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );

	GPIO_StructInit( &GPIO_InitStructure );
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init( GPIOA, &GPIO_InitStructure );

	///////////////////////////////////////////////////////////////////////////
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );

	GPIO_StructInit( &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin =
	(
		GPIO_Pin_13 	// MCU_DIR
	);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

	GPIO_StructInit( &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin =
	(
		GPIO_Pin_12 |	// MCU_DO0
		GPIO_Pin_15 	// MCU_BR
	);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;

	GPIO_Init( GPIOB, &GPIO_InitStructure );

	GPIO_ResetBits( GPIOB, GPIO_Pin_15 | GPIO_Pin_12 );

	///////////////////////////////////////////////////////////////////////////
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );

	GPIO_StructInit( &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin =
	(
			GPIO_Pin_6	// MCU_STEP
	);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;

	GPIO_Init( GPIOC, &GPIO_InitStructure );

	///////////////////////////////////////////////////////////////////////////
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );

	GPIO_StructInit( &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin =
	(
			GPIO_Pin_11 |	// MCU_CHARGE_RELAY
			GPIO_Pin_10		// MCU_EN_POWER_STAGE
	);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;

	GPIO_Init( GPIOD, &GPIO_InitStructure );

	GPIO_SetBits( GPIOD, GPIO_Pin_10 );
	GPIO_SetBits( GPIOD, GPIO_Pin_11 );
	///////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
}

eMBErrorCode eMBRegHoldingCB
(
	UCHAR * pucRegBuffer, USHORT usAddress,
	USHORT usNRegs, eMBRegisterMode eMode
)
{
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;

	if( ( usAddress >= REG_HOLDING_START ) &&
		( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS )
	) {
		iRegIndex = ( int )( usAddress - usRegHoldingStart );
		switch ( eMode ) {
		case MB_REG_READ:
        	while( usNRegs > 0 ) {
        		*pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] >> 8 );
        		*pucRegBuffer++ = ( UCHAR ) ( usRegHoldingBuf[iRegIndex] & 0xFF );
        		iRegIndex++;
        		usNRegs--;
        	}
         break;
		case MB_REG_WRITE:
			while( usNRegs > 0 ) {
				usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
				usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
				iRegIndex++;
				usNRegs--;
			}
		 break;
		}
	} else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress,
                            USHORT usNCoils, eMBRegisterMode eMode )
{
	return MB_ENOREG;
}

eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
	return MB_ENOREG;
}

eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
	return MB_ENOREG;
}
