#include "main.h"

#define REG_HOLDING_START   40001
#define REG_HOLDING_NREGS   50

extern float sp_speed, pv_speed;
extern int32_t sp_pos, pv_pos;
extern PID pidPos, pidSpeed;

extern int16_t tim8_overflow, enc_delta;
extern uint32_t uwTIM10PulseLength;
extern uint16_t sanyo_uvw;

extern float ai0_filtered_value;
extern uint16_t ai0, ai1;

extern float vel;

volatile MC_FOC stFoc;

static USHORT usRegHoldingStart = REG_HOLDING_START;
static USHORT usRegHoldingBuf[ REG_HOLDING_NREGS ] = { 0 };

int main(void)
{
	float Iq_filtered_value = 0, Iq_des_filtered_value = 0;
	float TIM10PulseLength_filtered_value = 0;
	float dc_bus_filtered_value = 0;
	int32_t rpm;

	uint32_t charge_relya_on_delay_counter = 0, charge_relya_on_delay = 200000;
	uint32_t foc_enable_on_delay_counter = 0, foc_enable_on_delay = 100000;

	eMBErrorCode eStatus;
	int hall, encoder;

	SystemInit();
	SystemCoreClockUpdate();

	boardInit();

	focInit( &stFoc );

	eStatus = eMBInit( MB_RTU, 1, 0, 115200, MB_PAR_NONE );
	if( MB_ENOERR == eStatus ) {
		eStatus = eMBEnable();
	}

	///////////////////////////////////////////////////////////////////////////
	uint32_t sum_a = 0, sum_b = 0;
	for( int i = 0; i < 100; i++ ) {
		for( volatile uint32_t i = 0; i < 100000; i++ );
		sum_a += stFoc.current_a;
		sum_b += stFoc.current_b;
	}

	stFoc.current_a_offset = sum_a / 100;
	stFoc.current_b_offset = sum_b / 100;
	///////////////////////////////////////////////////////////////////////////
	stFoc.main_state = 1;

	while( 1 ) {
		GPIO_ToggleBits( GPIOA, GPIO_Pin_15 );

		FirstOrderLagFilter( &Iq_des_filtered_value,  stFoc.Iq_des, 0.00005f );
		FirstOrderLagFilter( &Iq_filtered_value,  stFoc.Iq, 0.00005f );
		FirstOrderLagFilter( &dc_bus_filtered_value, stFoc.Ubus, 0.0002f );
		FirstOrderLagFilter( &ai0_filtered_value, (float)ai0, 0.009f );

		FirstOrderLagFilter( &TIM10PulseLength_filtered_value,  (float)uwTIM10PulseLength,  0.0005f );
		FirstOrderLagFilter( &stFoc.f_rpm_m_filtered_value, stFoc.f_rpm_m, 0.0005f );
		FirstOrderLagFilter( &stFoc.f_rpm_t_filtered_value, stFoc.f_rpm_t, 0.0005f );

		FirstOrderLagFilter( &stFoc.f_rpm_mt_filtered_value, stFoc.f_rpm_mt, 0.00005f );
		FirstOrderLagFilter( &stFoc.f_rpm_mt_temp_filtered_value, stFoc.f_rpm_mt, 0.00008f );

		if( dc_bus_filtered_value > 1000 ) {
			if( charge_relya_on_delay_counter >= charge_relya_on_delay ) {
				GPIO_SetBits( GPIOD, GPIO_Pin_11 ); // MCU_CHARGE_RELAY
				GPIO_SetBits( GPIOD, GPIO_Pin_10 ); // MCU_EN_POWER_STAGE

				if( foc_enable_on_delay_counter >= foc_enable_on_delay ) {
					TIM_Cmd( TIM8, ENABLE );
					stFoc.enable = 1;
				} else {
					++foc_enable_on_delay_counter;
				}
			} else {
				++charge_relya_on_delay_counter;
			}
		} else {
			if( dc_bus_filtered_value < 900 ) {
				GPIO_ResetBits( GPIOD, GPIO_Pin_11 ); // MCU_CHARGE_RELAY
				GPIO_ResetBits( GPIOD, GPIO_Pin_10 ); // MCU_EN_POWER_STAGE
				charge_relya_on_delay_counter = 0;
				stFoc.enable = 0;
			}
		}

		stFoc.Is = sqrtf( stFoc.Id * stFoc.Id + stFoc.Iq * stFoc.Iq );
		rpm = stFoc.f_rpm_mt_filtered_value*100.0f;
		//rpm = ( sp_pos - pv_pos ) * 100;
		//rpm = sp_pos * 100;

		//hall = readRawHallInput();
		hall = readHallMap();
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		usRegHoldingBuf[0] = (int)stFoc.Ia;
		usRegHoldingBuf[1] = (int)stFoc.Ib;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		usRegHoldingBuf[2] = (int)dc_bus_filtered_value;
		usRegHoldingBuf[3] = ai0_filtered_value;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		// Encoder 0 ( rot.angle )
		usRegHoldingBuf[4] = hall; // !!!
		usRegHoldingBuf[6] = TIM3->CNT;
		usRegHoldingBuf[5] = stFoc.angle*0.0879f;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		// Encoder 1 ( abs.pos )
		usRegHoldingBuf[7] = pv_pos;
		usRegHoldingBuf[8] = pv_pos>>16;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		usRegHoldingBuf[11] = (int16_t)enc_delta;
		usRegHoldingBuf[12] = (uint16_t)TIM10PulseLength_filtered_value;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		usRegHoldingBuf[13] = (int)stFoc.Is;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		usRegHoldingBuf[14] = rpm;
		usRegHoldingBuf[15] = rpm>>16;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		usRegHoldingBuf[16] = sp_pos;
		usRegHoldingBuf[17] = sp_pos>>16;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		usRegHoldingBuf[18] = (int)pidPos.error;
		usRegHoldingBuf[19] = (int)pidPos.error>>16;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		// temp:
		int temp;

		temp = (int)(vel*100.0f);

		usRegHoldingBuf[20] = (int)(temp);
		usRegHoldingBuf[21] = (int)(temp>>16);
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		usRegHoldingBuf[22] = (int)(stFoc.f_rpm_mt*100.0f);
		usRegHoldingBuf[23] = (int)(stFoc.f_rpm_mt*100.0f)>>16;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		usRegHoldingBuf[24] = (int)(sp_speed*100.0f);
		usRegHoldingBuf[25] = (int)(sp_speed*100.0f)>>16;
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
		(void)eMBPoll();
		// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	}
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
	//Configure_PC6();
	//Configure_PC7();

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
	///////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
}
