/*
 * Project: STMBL - https://github.com/rene-dev/stmbl
 * File: https://github.com/rene-dev/stmbl/blob/master/src/comps/endat.c
 *
 * endat.c
 *
 *  Created on: Nov 23, 2020
 */

#include "endat.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

const uint64_t bitmask[] = {
  0b0,
  0b1,
  0b11,
  0b111,
  0b1111,
  0b11111,
  0b111111,
  0b1111111,
  0b11111111,
  0b111111111,
  0b1111111111,
  0b11111111111,
  0b111111111111,
  0b1111111111111,
  0b11111111111111,
  0b111111111111111,
  0b1111111111111111,
  0b11111111111111111,
  0b111111111111111111,
  0b1111111111111111111,
  0b11111111111111111111,
  0b111111111111111111111,
  0b1111111111111111111111,
  0b11111111111111111111111,
  0b111111111111111111111111,
  0b1111111111111111111111111,
  0b11111111111111111111111111,
  0b111111111111111111111111111,
  0b1111111111111111111111111111,
  0b11111111111111111111111111111,
  0b111111111111111111111111111111,
  0b1111111111111111111111111111111,
  0b11111111111111111111111111111111,
  0b111111111111111111111111111111111,
  0b1111111111111111111111111111111111,
  0b11111111111111111111111111111111111,
  0b111111111111111111111111111111111111,
  0b1111111111111111111111111111111111111,
  0b11111111111111111111111111111111111111,
  0b111111111111111111111111111111111111111,
  0b1111111111111111111111111111111111111111,
  0b11111111111111111111111111111111111111111,
  0b111111111111111111111111111111111111111111,
  0b1111111111111111111111111111111111111111111,
  0b11111111111111111111111111111111111111111111,
  0b111111111111111111111111111111111111111111111,
  0b1111111111111111111111111111111111111111111111,
  0b11111111111111111111111111111111111111111111111,
  0b111111111111111111111111111111111111111111111111,
};

union
{
  uint64_t data;
  uint8_t dataa[8];
} df, df1, df2, df_pos;

endat_data_t en_data_value;
endat_data_t data;
endat_cmd_t req;
//////////////////////////////
extern SPI_HandleTypeDef hspi3;
//////////////////////////////
uint64_t shift = 0;

uint8_t error = 0;
uint8_t state = 0;

uint8_t pos_len = 17;
uint8_t mpos_len = 0;
uint8_t endat_state = 13;
uint8_t swap = 0;
uint8_t skip = 10;
uint8_t bytes = 7;

void en_dat_inti(void)
{
}

uint32_t endat_tx(endat_cmd_t cmd, uint8_t p1, uint16_t p2, uint8_t* buf, endat_data_t* data){
  uint32_t len = 0;

  buf[0] = flip8(cmd);

  buf[1] = flip8(p1);

  buf[2] = flip16(p2) & 0xff;
  buf[3] = (flip16(p2) >> 8) & 0xff;

  data->current_cmd = cmd;
  data->current_addr = p1;
  data->current_value = p2;

  switch(cmd){
    case ENDAT_READ_POS:
      len = 2 + 6;
    break;

    case ENDAT_SELECT_MEM:
      len = 2 + 6 + 8 + 16;
      data->current_mem = p1;
    break;

    case ENDAT_READ_ADDR:
      len = 2 + 6 + 8 + 16;
    break;

    case ENDAT_WRITE_ADDR:

      len = 2 + 6 + 8 + 16;
    break;

    case ENDAT_RESET:
      len = 2 + 6 + 8 + 16;
    break;

    default:
      len = 0;
  }

  return(len);
}

uint32_t endat_rx(uint8_t* buf, uint32_t max_len, endat_data_t* data)
{
  union{
    uint64_t data64;
    uint32_t data32[2];
    uint16_t data16[4];
    uint8_t data8[8];
  }df;

  uint32_t len = 0;

  endat_cmd_t cmd = data->current_cmd;
  data->current_cmd = 0;

  uint8_t addr = data->current_addr;
  data->current_addr = 0;

  uint16_t value = data->current_value;
  data->current_value = 0;

  switch(cmd){
    case ENDAT_READ_POS:
      len = 1 + 1 + data->pos_bits + data->mpos_bits + 5;
    break;

    case ENDAT_SELECT_MEM:
      len = 1 + 8 + 16 + 5;
    break;

    case ENDAT_READ_ADDR:
      len = 1 + 8 + 16 + 5;
    break;

    case ENDAT_WRITE_ADDR:
      len = 1 + 8 + 16 + 5;
    break;

    case ENDAT_RESET:
      len = 1 + 8 + 16 + 5;
    break;

    default:
      return(0);
  }

  if(max_len < len){ // too short
    return(0);
  }

  for(int i = 0; i < (len + 7) / 8; i++){ // local copy
    df.data8[i] = buf[i];
  }

  uint8_t error_bit = 0;
  uint64_t temp1 = 0;
  uint64_t temp2 = 0;
  uint8_t p1;
  uint16_t p2;

  switch(cmd){
    case ENDAT_READ_POS:
      error_bit = df.data8[0] & 0x10;

      df.data64 >>= 2; // remove start + error bit
      temp1 = df.data64 & bitmask[data->pos_bits];
      df.data64 >>= data->pos_bits; // remove pos
      temp2 = df.data64 & bitmask[data->mpos_bits];
      df.data64 >>= data->mpos_bits; // remove mutliturn pos
      data->crc = df.data64 & bitmask[5];

      //check crc
      data->error_bit = 0;
      if(error_bit){
        data->error_bit = 1;
        //return(0);
      }
      data->pos = temp1;
      data->mpos = temp2;
    break;

    case ENDAT_SELECT_MEM:
      p1 = (df.data16[0] >> 1) & 0xff;
      data->crc = (df.data8[3] >> 1) & 0b11111;
      p1 = flip8(p1);

      //check crc
      if(data->current_mem != p1){
        data->current_mem = 0;
        return(0);
      }
    break;

    case ENDAT_READ_ADDR:
      p1 = (df.data16[0] >> 1) & 0xff;
      p2 = (df.data32[0] >> (1 + 8)) & 0xffff;
      data->crc = (df.data8[3] >> 1) & 0b11111;
      p1 = flip8(p1);
      p2 = flip16(p2);

      //check crc
      if(addr != p1){
        return(0);
      }

      switch(data->current_mem){
        case ENDAT_MEM_STATE:
          switch(p1){
            case 0: // error register
              // data->error = *((endat_state_error_t*) &p2);
              data->error.reg = p2;
            break;

            case 1: // warning register
              // data->warning = *((endat_state_warning_t*) &p2);
              data->warning.reg = p2;
            break;

            default:
              return(0);
          }
        break;

        case ENDAT_MEM_PARAM0:
          switch(p1){
            case ENDAT_ADDR_POS_LEN:
              data->pos_len = p2;
              data->pos_bits = data->pos_len - data->mpos_bits;
            break;

            case ENDAT_ADDR_TYPE:
              data->fb_type = p2;
            break;

            default:
              return(0);
          }
        break;

        case ENDAT_MEM_PARAM1:
          switch(p1){
            case ENDAT_ADDR_MULTITURN:
              data->mpos_bits = log2f(p2) + 0.99;
              data->pos_bits = data->pos_len - data->mpos_bits;
            break;

            case ENDAT_ADDR_RES_LOW:
              data->pos_res = (data->pos_res & 0xffff0000) | p2;
            break;

            case ENDAT_ADDR_RES_HIGH:
              data->pos_res = (data->pos_res & 0xffff) | (p2 << 16);
            break;

            default:
              return(0);
          }
        break;

        case ENDAT_MEM_PARAM2:
          switch(p1){
            case ENDAT_ADDR_MAX_VEL:
              data->max_vel = p2;
            break;

            default:
              return(0);
          }
        break;

        default:
          return(0);
      }

    break;

    case ENDAT_WRITE_ADDR:
      p1 = (df.data16[0] >> 1) & 0xff;
      p2 = (df.data32[0] >> (1 + 8)) & 0xffff;
      data->crc = (df.data8[3] >> 1) & 0b11111;
      p1 = flip8(p1);
      p2 = flip16(p2);

      //check crc
      if(addr != p1){
        return(0);
      }
      if(value != p2){
        return(0);
      }
    break;

    case ENDAT_RESET:
      data->crc = (df.data8[3] >> 1) & 0b11111;

      //check crc
    break;
  }

  return(1);
}

volatile uint64_t post = 0;

void endat_func( float period, endat_data_t *ctx )
{
	uint8_t addr = 0;

	req = 0;
	ctx->pos_bits = pos_len;
	ctx->mpos_bits = mpos_len;

	endat_state = 13;

	switch( endat_state )
	{
	case 0: // reset error
		req = ENDAT_RESET;
	break;

	case 1: // select mem state
		req = ENDAT_SELECT_MEM;
		addr = ENDAT_MEM_STATE;
	break;

	case 2: // read error
		req = ENDAT_READ_ADDR;
		addr = ENDAT_ADDR_ERROR;
	break;

	case 3: // read warning
		req = ENDAT_READ_ADDR;
		addr = ENDAT_ADDR_WARNING;
	break;

    case 4: // select mem 0
      req = ENDAT_SELECT_MEM;
      addr = ENDAT_MEM_PARAM0;
    break;

    case 5: // read len
      req = ENDAT_READ_ADDR;
      addr = ENDAT_ADDR_POS_LEN;
    break;

    case 6: // read type
      req = ENDAT_READ_ADDR;
      addr = ENDAT_ADDR_TYPE;
    break;

    case 7: // select mem 1
      req = ENDAT_SELECT_MEM;
      addr = ENDAT_MEM_PARAM1;
    break;

    case 8: // read multi
      req = ENDAT_READ_ADDR;
      addr = ENDAT_ADDR_MULTITURN;
    break;

    case 9: // read res low
      req = ENDAT_READ_ADDR;
      addr = ENDAT_ADDR_RES_LOW;
    break;

    case 10: // read res high
      req = ENDAT_READ_ADDR;
      addr = ENDAT_ADDR_RES_HIGH;
    break;

    case 11: // select mem 2
      req = ENDAT_SELECT_MEM;
      addr = ENDAT_MEM_PARAM2;
    break;

    case 12: // read max vel
      req = ENDAT_READ_ADDR;
      addr = ENDAT_ADDR_MAX_VEL;
    break;

    case 13: // read pos
      req = ENDAT_READ_POS;
    break;
	}

	/*if( PIN(req) > 0 )
	{
		req = PIN(req);
	}*/

	uint32_t bits = endat_tx( req, addr, 0, df.dataa, ctx );
	df2.data = df.data;

	SPI3->CR1 &= ~SPI_CR1_SPE;
	if( swap )
		SPI3->CR1 |= SPI_CR1_CPHA;
	else
		SPI3->CR1 &= ~SPI_CR1_CPHA;

	GPIOD->BSRR |= ENDAT_TX_EN_OR_USART2_RTS_Pin; //tx enable
	HAL_SPI_Transmit( &hspi3, df.dataa, (bits + 7) / 8, 10);
	GPIOD->BSRR |= ENDAT_TX_EN_OR_USART2_RTS_Pin<<16; //tx disable

	if( swap )
	{
		//SPI3->CR1 = SPI_CR1_LSBFIRST | SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_BIDIMODE | SPI_BAUDRATEPRESCALER_32;
	} else{
		//SPI3->CR1 = SPI_CR1_LSBFIRST | SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_BIDIMODE | SPI_BAUDRATEPRESCALER_32;
	}
	//SPI3->CR1 |= SPI_CR1_SPE;//enable spi

	SPI3->CR1 &= ~SPI_CR1_SPE;
	if( !swap )
		SPI3->CR1 |= SPI_CR1_CPHA;
	else
		SPI3->CR1 &= ~SPI_CR1_CPHA;

	df.data = 0;
	HAL_SPI_Receive( &hspi3, df.dataa, MIN( sizeof(df.data), bytes ), 10);

	df1.data = df.data >> (int)skip;
	shift = skip;

	while( (df1.data & 1) == 0 && shift < 32.0 )
	{
		df1.data = df1.data >> 1;
		shift++;
	}

	if( df.data && ( df.dataa[0] == 193 ))
	{
		volatile uint16_t p = 0;

		//post = df1.data;
		df_pos = df;

		p  = df_pos.dataa[1];
		p |= df_pos.dataa[2]<<8;

		post = p;
	}

	uint32_t ret = endat_rx( df1.dataa, MIN( sizeof(df1.data), bytes ) * 8, ctx );

	switch( endat_state )
	{
	case 12:
		if( ctx->error_bit )
		{
			endat_state = 0;
			error = 1;
			state = 0;
		} else {
			state = 1;
			error = 0;
		}

		if( !ret )
		{
			endat_state = 0;
			error = 1;
			state = 0;
		}
	break;

	default:
		state = 0;
		if( !ret )
		{
			endat_state = 0;
		} else {
			endat_state++;
		}
	}

	en_data_value = *ctx;

	pos_len = ctx->pos_bits;
	mpos_len = ctx->mpos_bits;

	if( ctx->pos_bits )
	{
		volatile int64_t pos;

		pos = /*mod*/( ctx->pos * 2.0 * M_PI / (1 << ctx->pos_bits) );
	}

	/*if( PIN(print_time) > 0.0 )
	{
		PIN(timer) += period;
	}*/
}

void endat_main( endat_data_t *ctx )
{
	//if( PIN(timer) > PIN(print_time) )
	{
		//PIN(timer) = 0.0;

		uint64_t data = df.data;
		uint64_t reqdata = df2.data;
		uint64_t pos1 = ctx->pos;
		uint64_t pos2 = ctx->mpos;
		uint64_t crc = ctx->crc;
		uint32_t shift = shift;

		//printf("req: ");
		switch(req)
		{
		case ENDAT_SELECT_MEM:
			//printf("ENDAT_SELECT_MEM\n");
		break;

		case ENDAT_READ_ADDR:
			//printf("ENDAT_READ_ADDR\n");
		break;

		case ENDAT_WRITE_ADDR:
			//printf("ENDAT_WRITE_ADDR\n");
		break;

		case ENDAT_READ_POS:
			//printf("ENDAT_READ_POS\n");
		break;

		default:
			//printf("unkonwn req\n");
		break;
		}

		//printf("req data: ");
		for(int i = 0; i < 64; i++)
		{
			if( reqdata & 1 )
			{
				//printf("1");
			} else {
				//printf("0");
			}

			reqdata >>= 1;

			if( i == 8 - 1 )
			{
				//printf("-");
			}

			if( i == 8 + 8 - 1 )
			{
				//printf("-");
			}

			if( i == 8 + 8 + 16 - 1 )
			{
				//printf("-");
			}

			if( i == 8 + 8 + 16 + 5 - 1 )
			{
				//printf("-");
			}
		}
		//printf("\n");

		//printf("data: ");
		for(int i = 0; i < 32; i++)
		{
			if(data & 1)
			{
				//printf("1");
			} else {
				//printf("0");
			}

			data >>= 1;
			if(i == shift - 1)
			{
				//printf("-");
			}

			if(i == shift + 2 - 1)
			{
				//printf("-");
			}

			if( i == shift + 2 + ctx->pos_bits - 1 )
			{
				//printf("-");
			}

			if( i == shift + 2 + ctx->pos_bits + ctx->mpos_bits - 1 )
			{
				//printf("-");
			}
		}

		//printf("\npos1: ");
		for(int i = 0; i < 64; i++)
		{
			if( pos1 & 1 )
			{
				//printf("1");
			} else {
				//printf("0");
			}

			pos1 >>= 1;
			if(i == ctx->pos_bits - 1)
			{
				//printf("-");
			}
		}

		//printf("\npos2: ");
		for(int i = 0; i < 64; i++)
		{
			if(pos2 & 1)
			{
				//printf("1");
			} else {
				//printf("0");
			}

			pos2 >>= 1;
			if( i == ctx->mpos_bits - 1 )
			{
				//printf("-");
			}
		}

		//printf("\ncrc:  ");
		for( int i = 0; i < 64; i++ )
		{
			if( crc & 1 )
			{
				//printf("1");
			} else {
				//printf("0");
			}

			crc >>= 1;
			if(i == 5 - 1)
			{
				//printf("-");
			}
		}

		//printf("\n\n");
	}
}

inline uint8_t flip8(uint8_t d)
{
  uint8_t r = d & 1;
  for(int i = 0; i < sizeof(r) * 8 - 1; i++){
    r <<= 1;
    d >>= 1;
    r |= d & 1;
  }

  return(r);
}

inline uint16_t flip16(uint16_t d)
{
  uint16_t r = d & 1;
  for(int i = 0; i < sizeof(r) * 8 - 1; i++){
    r <<= 1;
    d >>= 1;
    r |= d & 1;
  }

  return(r);
}

inline uint32_t flip32(uint32_t d)
{
  uint32_t r = d & 1;
  for(int i = 0; i < sizeof(r) * 8 - 1; i++){
    r <<= 1;
    d >>= 1;
    r |= d & 1;
  }

  return(r);
}

inline uint64_t flip64(uint64_t d)
{
  uint64_t r = d & 1;
  for(int i = 0; i < sizeof(r) * 8 - 1; i++){
    r <<= 1;
    d >>= 1;
    r |= d & 1;
  }

  return(r);
}
