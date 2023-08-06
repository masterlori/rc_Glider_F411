/**
 * @defgroup  leds_interface
 * @ref		  leds_interface
 * @author    Neat Tech
 * @brief     Software layer to handle communication between the application and the configurator app
 *
 * @{
*/

#include "main.h"
#include "modem_interface.h"
#include "../crc/crc.h"
#include "../system/system.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

volatile modem_infoTypeDef modem_info;

/*modem states*/
typedef enum
{
	modem_init_power_up,
	modem_init_set_par,
	modem_init_read_par,
	modem_init_wait_par,
	modem_init_wait_rdy,
	modem_ready
} modemstate_TypeDef;
modemstate_TypeDef _modem_cur_state = modem_init_power_up;

uint8_t _modem_par_buf[10];
uint8_t _modem_par_buf_cnt = 0;
uint8_t _modem_par_flag = 0;

uint8_t _modem_led_start = 0;
uint8_t _modem_led_state = 0;


void modem_Timer(uint32_t res)
{
#ifdef NO_MODEM
	return;
#endif //NO_MODEM

	for( uint8_t i = 0; i < MODEM_TMR_NUM; i++ )
	{
		if( modem_info.timer[i] > 0 ){
			modem_info.timer[i]--;
		}
	}

	return;
}

/**
  * @brief  called from the main before loop
  * @param  None
  * @retval None
  */
/**********************************************************************/
void modem_InitTask()
{
#ifdef NO_MODEM
	return;
#endif //NO_MODEM

	HAL_GPIO_WritePin(GPIOA, MODEM_M0_Pin|MODEM_M1_Pin, GPIO_PIN_SET);
	modem_info.ready_flag = 0;

	modem_info.timer[MODEM_TX_TMR] = 0;
	modem_info.rx_buf.head = 0;
	modem_info.rx_buf.tail = 0;
	modem_info.rx_buf.state = MDMRINGBUF_WAIT_HEADER;
	modem_info.tx_buf_bytes = 0;

	return;
}

/**
  * @brief  called from the main loop
  * @param  None
  * @retval None
  */
/**********************************************************************/
void modem_Task(void)
{
#ifdef NO_MODEM
	return;
#endif //NO_MODEM

	uint8_t set_buf[10];

	if( _modem_cur_state == modem_init_power_up )
	{
		if( HAL_GPIO_ReadPin(MODEM_AUX_GPIO_Port, MODEM_AUX_Pin) == GPIO_PIN_SET )
		{
			modem_info.timer[MODEM_STATE_TMR] = 1000;
			_modem_cur_state = modem_init_set_par;
		}
	}
	else if( _modem_cur_state == modem_init_set_par )
	{
		if( modem_info.timer[MODEM_STATE_TMR] == 0 )
		{
			//Command "Set parameters without saving"
			set_buf[0] = 0xC2;
			//Address (Broadcast)
			set_buf[1] = 0xFF;
			set_buf[2] = 0xFF;
			//Speed (8N1, 115200, RF-19.2k)
			set_buf[3] = 0x3D;
			//Channel (868 MHz)
			set_buf[4] = 0x06;
			//Options
			set_buf[5] = 0x44;
			HAL_UART_Transmit(&HAL_MODEM_UART, set_buf, 6, HAL_MAX_DELAY);
			modem_info.timer[MODEM_STATE_TMR] = 100;
			_modem_cur_state = modem_init_read_par;
		}
	}
	else if( _modem_cur_state == modem_init_read_par )
	{
		if( modem_info.timer[MODEM_STATE_TMR] == 0 )
		{
			//Command "Read parameters"
			set_buf[0] = 0xC1;
			set_buf[1] = 0xC1;
			set_buf[2] = 0xC1;
			HAL_UART_Transmit(&HAL_MODEM_UART, set_buf, 3, HAL_MAX_DELAY);
			_modem_cur_state = modem_init_wait_par;
		}
	}
	else if( _modem_cur_state == modem_init_wait_par )
	{
		if( _modem_par_flag == 1 )
		{
			_modem_par_flag = 0;
			HAL_GPIO_WritePin(GPIOA, MODEM_M0_Pin|MODEM_M1_Pin, GPIO_PIN_RESET);
			system_UARTmodemSet115200();
			modem_info.timer[MODEM_STATE_TMR] = 100;
			_modem_cur_state = modem_init_wait_rdy;
		}
	}
	else if( _modem_cur_state == modem_init_wait_rdy  )
	{
		if( modem_info.timer[MODEM_STATE_TMR] == 0 ){
			_modem_cur_state = modem_ready;
		}
	}
	else if( _modem_cur_state == modem_ready )
	{
		//Transmitting
		if( (modem_info.tx_buf_bytes > 0) && (modem_info.timer[MODEM_TX_TMR] == 0) )
		{
			modem_info.timer[MODEM_TX_TMR] = MODEM_TX_PERIOD;
			if( modem_info.tx_buf_bytes > MODEM_MAX_TX_LEN )
			{
				HAL_UART_Transmit(&HAL_MODEM_UART, (uint8_t*)modem_info.tx_buf, MODEM_MAX_TX_LEN, HAL_MAX_DELAY);
				modem_RemTxData(MODEM_MAX_TX_LEN);
			}
			else
			{
				HAL_UART_Transmit(&HAL_MODEM_UART, (uint8_t*)modem_info.tx_buf, modem_info.tx_buf_bytes, HAL_MAX_DELAY);
				modem_info.tx_buf_bytes = 0;
			}
		}

		//Receiving
		if( modem_info.rvd_pkt_size != 0 )
		{
			_modem_led_start = 1;
			system_ModemRxCallback(modem_info.rvd_pkt[0], (uint8_t*)&modem_info.rvd_pkt[1], modem_info.rvd_pkt_size - 1);
			modem_info.rvd_pkt_size = 0;
		}
	}

	//LED
	if( _modem_led_state == 0)
	{
		if( _modem_led_start == 1 )
		{
			_modem_led_start = 0;
			_modem_led_state = 1;
		}
	}
	else if( _modem_led_state == 1 )
	{
		HAL_GPIO_WritePin(MODEM_LED_GPIO_Port, MODEM_LED_Pin, GPIO_PIN_SET);
		modem_info.timer[MODEM_LED_TMR] = 20;
		_modem_led_state = 2;
	}
	else if( _modem_led_state == 2 )
	{
		if( modem_info.timer[MODEM_LED_TMR] == 0 )
		{
			HAL_GPIO_WritePin(MODEM_LED_GPIO_Port, MODEM_LED_Pin, GPIO_PIN_RESET);
			modem_info.timer[MODEM_LED_TMR] = 50;
			_modem_led_state = 3;
		}
	}
	else if( _modem_led_state == 3 )
	{
		if( modem_info.timer[MODEM_LED_TMR] == 0 ){
			_modem_led_state = 0;
		}
	}

	return;
}

void modem_RcvData(uint8_t data)
{
#ifdef NO_MODEM
	return;
#endif //NO_MODEM

	static uint16_t rcv_size;
	uint16_t header;
	uint16_t crc;

	if( _modem_cur_state != modem_ready )
	{
		if( _modem_par_flag == 0 )
		{
			_modem_par_buf[_modem_par_buf_cnt++] = data;
			if( _modem_par_buf_cnt == 6 )
			{
				_modem_par_buf_cnt = 0;
				_modem_par_flag = 1;
			}
		}
	}
	else
	{
		modem_info.rx_buf.data[modem_info.rx_buf.tail =
				(modem_info.rx_buf.tail + 1) & MODEM_BUF_MASK] = data;
		//Waiting header
		if( modem_info.rx_buf.state == MDMRINGBUF_WAIT_HEADER )
		{
			//Catch header
			header = modem_ringBufRead16b((uint8_t*)modem_info.rx_buf.data,
					(modem_info.rx_buf.tail - 2) & MODEM_BUF_MASK);
			if( header == MODEM_HEADER )
			{
				modem_info.rx_buf.head = (modem_info.rx_buf.tail - 2) & MODEM_BUF_MASK;
				rcv_size = data;
				modem_info.rx_buf.data_size  = data - 2;
				modem_info.rx_buf.state = MDMRINGBUF_WAIT_DATA;
			}
		}
		//Waiting data
		else if( modem_info.rx_buf.state == MDMRINGBUF_WAIT_DATA )
		{
			if( --rcv_size == 0 )
			{
				modem_info.rx_buf.state = MDMRINGBUF_WAIT_HEADER;
				//CRC check
				crc = crc16_RingBuf((uint8_t*)modem_info.rx_buf.data, modem_info.rx_buf.head,
						modem_info.rx_buf.data_size + 3, MODEM_BUF_MASK);
				if( crc == modem_ringBufRead16b((uint8_t*)modem_info.rx_buf.data,
						(modem_info.rx_buf.tail - 1) & MODEM_BUF_MASK) ){
					//Copy received pkt
					if( modem_info.rvd_pkt_size == 0 )
					{
						for( uint16_t i = 0; i < modem_info.rx_buf.data_size; i++ ){
							modem_info.rvd_pkt[i] =
									modem_info.rx_buf.data[(modem_info.rx_buf.head + 3 + i) & MODEM_BUF_MASK];
						}
						modem_info.rvd_pkt_size = modem_info.rx_buf.data_size;
					}
				}
			}
			else
			{

			}
		}
	}

	return;
}

void modem_TrmData(uint8_t channel, uint8_t *data, uint32_t len)
{
	uint16_t u16_tmp;
	uint8_t u8_tmp;
	uint32_t i;

	//Header
	u16_tmp = MODEM_HEADER;
	modem_AddTxData((uint8_t*)&u16_tmp, 2);
	//Length (data + channel + crc)
	u8_tmp = (uint8_t)len + 3;
	modem_AddTxData(&u8_tmp, 1);
	//Channel (starts from 1)
	u8_tmp = channel;
	modem_AddTxData(&u8_tmp, 1);
	//Data
	modem_AddTxData(data, len);
	//CRC
	u16_tmp = crc16_calc(modem_info.tx_buf, modem_info.tx_buf_bytes);
	modem_AddTxData((uint8_t*)&u16_tmp, 2);

	return;
}

uint16_t modem_ringBufRead16b(uint8_t *buf, uint16_t pos)
{
	uint16_t tmp;
	tmp = buf[(pos + 1) & MODEM_BUF_MASK] & 0xFF;
	tmp = (tmp << 8) + buf[pos];
	return tmp;
}

uint32_t modem_ringBufRead32b(uint8_t *buf, uint16_t pos)
{
	uint32_t tmp;
	tmp = buf[(pos + 3) & MODEM_BUF_MASK] & 0xFF;
	tmp = (tmp << 8) + buf[(pos + 2) & MODEM_BUF_MASK];
	tmp = (tmp << 8) + buf[(pos + 1) & MODEM_BUF_MASK];
	tmp = (tmp << 8) + buf[pos];
	return tmp;
}

void modem_AddTxData(uint8_t *data, uint32_t len)
{
#ifdef NO_MODEM
	return;
#endif //NO_MODEM

	if( len > (MODEM_BUF_SIZE - modem_info.tx_buf_bytes) ){
		return;
	}
	memcpy((uint8_t*)&modem_info.tx_buf[modem_info.tx_buf_bytes], data, len);
	modem_info.tx_buf_bytes += len;

	return;
}

void modem_RemTxData(uint32_t len)
{
#ifdef NO_MODEM
	return;
#endif //NO_MODEM

	uint8_t tmp_buf[MODEM_BUF_SIZE];

	if( (len > modem_info.tx_buf_bytes)
			|| (len == 0) || (modem_info.tx_buf_bytes == 0) ){
		return;
	}
	modem_info.tx_buf_bytes -= len;
	memcpy(tmp_buf, (uint8_t*)&modem_info.tx_buf[len], modem_info.tx_buf_bytes);
	memcpy((uint8_t*)modem_info.tx_buf, tmp_buf, modem_info.tx_buf_bytes);

	return;
}


