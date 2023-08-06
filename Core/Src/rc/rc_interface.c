
#include "main.h"
#include "rc_interface.h"
#include "../configurator/cfg_interface.h"
#include "../crc/crc.h"
#include "../battery/battery_interface.h"
#include "../system/system.h"
#include "../modem/modem_interface.h"
#include "../autopilot/autopilot.h"
#include <stdlib.h>
#include <string.h>

volatile rc_infoTypeDef rc_info;
#define RC_TX_BUF_SIZE	128
uint8_t _rc_tx_buf[RC_TX_BUF_SIZE];
uint32_t _rc_tx_buf_len = 0;

uint16_t rc_ringBufRead16b(uint8_t *buf, uint16_t pos);
void rc_AddTxData(uint8_t *data, uint32_t len);

void rc_Timer(uint32_t res)
{
#ifdef NO_RC
	return;
#endif //NO_RC

	for( uint8_t i = 0; i < RC_TMR_NUM; i++ )
	{
		if( rc_info.timer[i] > 0 ){
			rc_info.timer[i]--;
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
void rc_InitTask()
{
#ifdef NO_RC
	return;
#endif //NO_RC

	rc_info.enable = 1;
	rc_info.mode = 0;
	rc_info.rx_pkt_cnt = 0;
	rc_info.conn_tout = RC_DEF_CONN_TOUT;
	rc_info.send_tlm = 1;

	rc_info.rx_buf.head = 0;
	rc_info.rx_buf.tail = 0;
	rc_info.rx_buf.state = RCRINGBUF_WAIT_HEADER;

	return;
}

/**
  * @brief  called from the main loop
  * @param  None
  * @retval None
  */
/**********************************************************************/
void rc_Task(void)
{
#ifdef NO_RC
	return;
#endif //NO_RC

	uint8_t tmp_u8;
	uint16_t tmp_u16;

	//Receiving
	if( rc_info.rvd_pkt_size != 0 ){
		rc_info.timer[RC_TMR_CONN] = rc_info.conn_tout;
		if( rc_info.connected == 0 ){
			rc_info.connected = 1;
		}

		rc_info.axis_l_x = (int8_t)rc_info.rvd_pkt[0];
		rc_info.axis_l_y = (int8_t)rc_info.rvd_pkt[1];
		rc_info.axis_r_x = (int8_t)rc_info.rvd_pkt[2];
		rc_info.axis_r_y = (int8_t)rc_info.rvd_pkt[3];
		rc_info.button_l = rc_info.rvd_pkt[4];
		rc_info.button_r = rc_info.rvd_pkt[5];

		rc_info.rx_pkt_cnt += 1;
		rc_info.rvd_pkt_size = 0;

	}

	//Connection tout condition
	if( (rc_info.timer[RC_TMR_CONN] == 0) && (rc_info.connected == 1) ){
		rc_info.connected = 0;
	}

	//Transmitting
	if( (rc_info.timer[RC_TMR_TRM_TLM] == 0)
			&& (rc_info.send_tlm == 1) && (rc_info.connected == 1) )
	{
		rc_info.timer[RC_TMR_TRM_TLM] = RC_TLM_SND_PERIOD;
		tmp_u16 = RC_HEADER;
		rc_AddTxData((uint8_t*)&tmp_u16, 2);
		tmp_u8 = 5;
		rc_AddTxData(&tmp_u8, 1);
		tmp_u8 = autopilot_info.armed_flag;
		rc_AddTxData(&tmp_u8, 1);
		tmp_u16 = battery_info.battery_volatge;
		rc_AddTxData((uint8_t*)&tmp_u16, 2);
		tmp_u16 = crc16_calc(_rc_tx_buf, _rc_tx_buf_len);
		rc_AddTxData((uint8_t*)&tmp_u16, 2);
		//modem_TrmData(SYSTEM_RC_MODEM_CH, _rc_tx_buf, _rc_tx_buf_len);
		_rc_tx_buf_len = 0;
	}

	return;
}

void rc_RcvData(uint8_t data)
{
#ifdef NO_RC
	return;
#endif //NO_RC

	static uint16_t rcv_size;
	uint16_t header;
	uint16_t crc;

	rc_info.rx_buf.data[rc_info.rx_buf.tail =
			(rc_info.rx_buf.tail + 1) & RC_BUF_MASK] = data;
	//Waiting header
	if( rc_info.rx_buf.state == RCRINGBUF_WAIT_HEADER )
	{
		//Catch header
		header = rc_ringBufRead16b((uint8_t*)rc_info.rx_buf.data,
				(rc_info.rx_buf.tail - 2) & RC_BUF_MASK);
		if( header == RC_HEADER )
		{
			rc_info.rx_buf.head = (rc_info.rx_buf.tail - 2) & RC_BUF_MASK;
			rcv_size = data;
			rc_info.rx_buf.data_size  = data - 2;
			rc_info.rx_buf.state = RCRINGBUF_WAIT_DATA;
		}
	}
	//Waiting data
	else if( rc_info.rx_buf.state == RCRINGBUF_WAIT_DATA )
	{
		if( --rcv_size == 0 )
		{
			rc_info.rx_buf.state = RCRINGBUF_WAIT_HEADER;
			//CRC check
			crc = crc16_RingBuf((uint8_t*)rc_info.rx_buf.data, rc_info.rx_buf.head,
					rc_info.rx_buf.data_size + 3, RC_BUF_MASK);
			if( crc == rc_ringBufRead16b((uint8_t*)rc_info.rx_buf.data,
					(rc_info.rx_buf.tail - 1) & RC_BUF_MASK) ){
				//Copy received pkt
				if( rc_info.rvd_pkt_size == 0 )
				{
					for( uint16_t i = 0; i < rc_info.rx_buf.data_size; i++ ){
						rc_info.rvd_pkt[i] =
								rc_info.rx_buf.data[(rc_info.rx_buf.head + 3 + i) & RC_BUF_MASK];
					}
					rc_info.rvd_pkt_size = rc_info.rx_buf.data_size;
				}
			}
		}
		else
		{

		}
	}

	return;
}

uint16_t rc_ringBufRead16b(uint8_t *buf, uint16_t pos)
{
	uint16_t tmp;
	tmp = buf[(pos + 1) & RC_BUF_MASK] & 0xFF;
	tmp = (tmp << 8) + buf[pos];

	return tmp;
}

void rc_AddTxData(uint8_t *data, uint32_t len)
{

	if( len > (RC_TX_BUF_SIZE - _rc_tx_buf_len) ){
		return;
	}
	memcpy((uint8_t*)&_rc_tx_buf[_rc_tx_buf_len], data, len);
	_rc_tx_buf_len += len;

	return;
}



/* Configurator node functions*/
/*************************************************************************/
uint16_t cfg_NodeRcVarProp(uint16_t varid, char *name, uint16_t *prop)
{
	char *str;

	switch( varid )
	{
		case RC_ENA:				str = "Enable"; break;
		case RC_MODE:				str = "Mode (0:modem 1:futaba)"; break;
		case RC_SEND_TLM:			str = "Send telemetry"; break;
		case RC_RX_PKT_CNT:			str = "Received pkt"; break;
		case RC_AXIS_L_X:			str = "Axis left X"; break;
		case RC_AXIS_L_Y:			str = "Axis left Y"; break;
		case RC_AXIS_R_X:			str = "Axis right X"; break;
		case RC_AXIS_R_Y:			str = "Axis right Y"; break;
		case RC_BUT_L:				str = "Button left"; break;
		case RC_BUT_R:				str = "Button right"; break;
		default: return CFG_ERROR_VARID;
	}
	if( name ) { while( *str ) *name++ = *str++; *name = 0; }

	if( prop ) switch( varid )
	{
		case RC_ENA:				*prop = CFG_VAR_TYPE_BOOL | CFG_VAR_PROP_CONST; break;
		case RC_MODE:				*prop = CFG_VAR_TYPE_UINT| CFG_VAR_PROP_CONST; break;
		case RC_SEND_TLM:			*prop = CFG_VAR_TYPE_BOOL | CFG_VAR_PROP_CONST; break;
		case RC_RX_PKT_CNT:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_READONLY; break;
		case RC_AXIS_L_X:			*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case RC_AXIS_L_Y:			*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case RC_AXIS_R_X:			*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case RC_AXIS_R_Y:			*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case RC_BUT_L:				*prop = CFG_VAR_TYPE_BOOL | CFG_VAR_PROP_READONLY; break;
		case RC_BUT_R:				*prop = CFG_VAR_TYPE_BOOL | CFG_VAR_PROP_READONLY; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeRcVarGet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case RC_ENA: 				*(uint32_t*)value = (uint32_t)rc_info.enable; break;
		case RC_MODE: 				*(uint32_t*)value = (uint32_t)rc_info.mode; break;
		case RC_SEND_TLM: 			*(uint32_t*)value = (uint32_t)rc_info.send_tlm; break;
		case RC_RX_PKT_CNT: 		*(uint32_t*)value = (uint32_t)rc_info.rx_pkt_cnt; break;
		case RC_AXIS_L_X: 			*(uint32_t*)value = (int32_t)rc_info.axis_l_x; break;
		case RC_AXIS_L_Y: 			*(uint32_t*)value = (int32_t)rc_info.axis_l_y; break;
		case RC_AXIS_R_X: 			*(uint32_t*)value = (int32_t)rc_info.axis_r_x; break;
		case RC_AXIS_R_Y: 			*(uint32_t*)value = (int32_t)rc_info.axis_r_y; break;
		case RC_BUT_L: 				*(uint32_t*)value = (uint32_t)rc_info.button_l; break;
		case RC_BUT_R: 				*(uint32_t*)value = (uint32_t)rc_info.button_r; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeRcVarSet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case RC_ENA:				rc_info.enable = (uint8_t)*(uint32_t*)value; break;
		case RC_MODE:				rc_info.mode = (uint8_t)*(uint32_t*)value; break;
		case RC_SEND_TLM:			rc_info.send_tlm = (uint8_t)*(uint32_t*)value; break;
		case RC_RX_PKT_CNT:			break;
		case RC_AXIS_L_X:			break;
		case RC_AXIS_L_Y:			break;
		case RC_AXIS_R_X:			break;
		case RC_AXIS_R_Y:			break;
		case RC_BUT_L:				break;
		case RC_BUT_R:				break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}


