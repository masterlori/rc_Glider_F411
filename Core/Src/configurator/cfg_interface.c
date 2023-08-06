/**
 * @defgroup  leds_interface
 * @ref		  leds_interface
 * @author    Neat Tech
 * @brief     Software layer to handle communication between the application and the configurator app
 *
 * @{
*/

#include "main.h"
#include "cfg_interface.h"
#include "../crc/crc.h"
#include "../system/system.h"
#include "../modem/modem_interface.h"
#include "../servo/servo_interface.h"
#include "../motor/motor_interface.h"
#include "../autopilot/autopilot.h"
#include "../rc/rc_interface.h"
#include "../battery/battery_interface.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

volatile cfg_infoTypeDef cfg_info;
volatile uint32_t cfg_time_counter;
volatile uint32_t cfg_new_fw_cur_addr;
volatile uint32_t cfg_new_fw_size;

/* @brief save parameters flash memory reservation */
__attribute__ ((section(".param_section"), used)) uint32_t param_arr[CFG_MAX_SAVE_PAR_NUM];

/* @brief new firmware flash memory reservation */
__attribute__ ((section(".new_firmware"), used)) uint32_t new_fw_start;

/* @brief nodes IDs, set/get functions*/
//Main node
#define	NODE_MAIN	1
uint16_t cfg_NodeMainVarProp(uint16_t varid, char *name, uint16_t *prop);
uint16_t cfg_NodeMainVarGet(uint16_t varid, void *value);
uint16_t cfg_NodeMainVarSet(uint16_t varid, void *value);

//Firmware node
#define	NODE_FW	101
uint16_t cfg_NodeFwVarProp(uint16_t varid, char *name, uint16_t *prop);
uint16_t cfg_NodeFwVarGet(uint16_t varid, void *value);
uint16_t cfg_NodeFwVarSet(uint16_t varid, void *value);

//Settings node
#define	NODE_DEV		102

//Servo node
#define	NODE_SERVO		10201

//Motor node
#define	NODE_MOTOR		10202

//RC node
#define	NODE_RC			10203

//RC node
#define	NODE_BAT		10204

//Autopilot node
#define	NODE_AUTOPILOT	103



/* @brief nodes parameters lists*/
//Main node
enum
{
	MAIN_DEVICE_ID,
	MAIN_WORKING_TIME,
	MAIN_SAVE,
	MAIN_RESET,
	MAIN_VAR_NUM
};


/* @brief list of nodes */
cgf_nodeTypeDef cfg_NodeList[] =
{
	{ NODE_MAIN, NODE_MAIN, MAIN_VAR_NUM, cfg_NodeMainVarGet, cfg_NodeMainVarSet, cfg_NodeMainVarProp },
	{ NODE_FW, NODE_MAIN, CFG_FIRMWARE_VAR_NUM, cfg_NodeFwVarGet, cfg_NodeFwVarSet, cfg_NodeFwVarProp },
	{ NODE_DEV, NODE_MAIN, 0, NULL, NULL, NULL},
	{ NODE_SERVO, NODE_DEV, SERVO_VAR_NUM, cfg_NodeServoVarGet, cfg_NodeServoVarSet, cfg_NodeServoVarProp},
	{ NODE_MOTOR, NODE_DEV, MOTOR_VAR_NUM, cfg_NodeMotorVarGet, cfg_NodeMotorVarSet, cfg_NodeMotorVarProp},
	{ NODE_RC, NODE_DEV, RC_VAR_NUM, cfg_NodeRcVarGet, cfg_NodeRcVarSet, cfg_NodeRcVarProp},
	{ NODE_BAT, NODE_DEV, BAT_VAR_NUM, cfg_NodeBatVarGet, cfg_NodeBatVarSet, cfg_NodeBatVarProp},
	{ NODE_AUTOPILOT, NODE_MAIN, AUTOPILOT_VAR_NUM, cfg_NodeApVarGet, cfg_NodeApVarSet, cfg_NodeApVarProp}
};

/* @brief get the name of the node */
uint16_t cfg_GetNodeName(uint16_t nodeid, char *name)
{
	char *str;

	// получаем имя переменной
	switch( nodeid )
	{
		case NODE_MAIN		: str = "Nimbus_2000"; break;
		case NODE_FW		: str = "Firmware"; break;
		case NODE_DEV    	: str = "Aircraft devices"; break;
		case NODE_SERVO    	: str = "Servo"; break;
		case NODE_MOTOR    	: str = "Motor"; break;
		case NODE_RC    	: str = "RC"; break;
		case NODE_BAT    	: str = "Battery"; break;
		case NODE_AUTOPILOT	: str = "Autopilot"; break;
		default:break;
	}
	if( name )
	{
		while( *str ){
			*name++ = *str++;
		}
		*name = 0;
	}

	return CFG_ERROR_NONE;
}

/* @brief get node*/
cgf_nodeTypeDef* cfg_GetNode(uint16_t nodeid)
{
	cgf_nodeTypeDef *node = cfg_NodeList;
	uint16_t n;

	for( n = 0; n < cfg_info.node_num && node->id != nodeid ; n++, node++ );

	return n == cfg_info.node_num ? 0 : node;
}

/* Main node functions*/
/*************************************************************************/
uint16_t cfg_NodeFwVarProp(uint16_t varid, char *name, uint16_t *prop)
{
	char *str;// = malloc(50);

	if( varid == 0 )
	{
		str = "Version";
		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY;
	}
	else if( varid == 1 )
	{
		str = "Prepare for flashing";
		*prop = CFG_VAR_TYPE_BOOL;
	}
	else
	{
		str = "Data";
		//str = malloc(15);
		//sprintf(str, "Data %u", varid - 2);
		*prop = CFG_VAR_TYPE_HEX | CFG_VAR_PROP_READONLY;
	}
	if( name ) { while( *str ) *name++ = *str++; *name = 0; }
	//free(str);
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeFwVarGet(uint16_t varid, void *value)
{
	if( varid == 0 ){
		*(float*)value = 0.9;
	}
	else if( varid == 1){
		*(uint32_t*)value = 0;
	}
	else {
		*(uint32_t*)value = 0xFFFFFFFF;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeFwVarSet(uint16_t varid, void *value)
{
	//Prepare for flashing (clear new fw buffer)
	if( varid == 1 )
	{
		if( *(uint32_t*)value != 0 ){
			cfg_ClearFwBuf();
		}
	}
	//Receiving data
	else if( varid < CFG_FW_VAR_NUM ){
		cfg_WriteFwData(*(uint32_t*)value);
	}
	//Receiving complete
	else if( varid == (CFG_FW_VAR_NUM + 1) ){
		cfg_info.new_fw_received = 1;
	}
	return CFG_ERROR_NONE;
}

/* Main node functions*/
/*************************************************************************/
uint16_t cfg_NodeMainVarProp(uint16_t varid, char *name, uint16_t *prop)
{
	char *str;

	switch( varid )
	{
		case MAIN_DEVICE_ID:			str = "Device ID"; break;
		case MAIN_WORKING_TIME:			str = "Working time"; break;
		case MAIN_SAVE:					str = "Save settings"; break;
		case MAIN_RESET:				str = "Reboot device"; break;
		default: return CFG_ERROR_VARID;
	}
	if( name ) { while( *str ) *name++ = *str++; *name = 0; }

	if( prop ) switch( varid )
	{
		case MAIN_DEVICE_ID:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_CONST; break;
		case MAIN_WORKING_TIME:	  		*prop = CFG_VAR_TYPE_TIME | CFG_VAR_PROP_READONLY; break;
		case MAIN_SAVE:					*prop = CFG_VAR_TYPE_BOOL; break;
		case MAIN_RESET:				*prop = CFG_VAR_TYPE_BOOL; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeMainVarGet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case MAIN_DEVICE_ID: 			*(uint32_t*)value = (uint32_t)cfg_info.device_id; break;
		case MAIN_WORKING_TIME: 		*(uint32_t*)value = HAL_GetTick(); break;
		case MAIN_SAVE: 				*(uint32_t*)value = 0; break;
		case MAIN_RESET: 				*(uint32_t*)value = 0; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeMainVarSet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case MAIN_DEVICE_ID:				cfg_info.device_id = *(uint16_t*)value; break;
		case MAIN_WORKING_TIME:				break;
		case MAIN_SAVE:						if( *(uint32_t*)value ) {cfg_SaveSettings();} break;
		case MAIN_RESET:					if( *(uint32_t*)value ) {system_info.reset_req = 1;} break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

/*************************************************************************/

void cfg_TxDataCh0(uint8_t *data, uint32_t len);

/**
  * @brief  called from SysTick_Handler to update local class time
  * @param  res - time pass in mSec
  * @retval None
  */
void cfg_Timer(uint32_t res)
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	for( uint32_t i = 0; i < CFG_IFACE_NUM; i++ )
	{
		if( cfg_info.iface[i].timer > 0 ){
			cfg_info.iface[i].timer--;
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
void cfg_InitTask()
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	cfg_info.device_id = 1;
	cfg_info.node_num = sizeof( cfg_NodeList ) / sizeof( cgf_nodeTypeDef );

	for( uint32_t i = 0; i < CFG_IFACE_NUM; i++ )
	{
		cfg_info.iface[i].timer = 0;
		cfg_info.iface[i].rx_buf.head = 0;
		cfg_info.iface[i].rx_buf.tail = 0;
		cfg_info.iface[i].rx_buf.state = RINGBUF_WAIT_HEADER;
		cfg_info.iface[i].tx_buf_bytes = 0;
		cfg_info.iface[i].req_num = 0;
	}

	cfg_info.iface[CFG_IFACE_CH0].tx_func = cfg_TxDataCh0;

	cfg_LoadSettings();

	return;
}

/**
  * @brief  called from the main loop
  * @param  None
  * @retval None
  */
/**********************************************************************/
void cfg_Task(void)
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	uint16_t tmp_u16;
	uint8_t tmp_u8;
	uint16_t node_num = 0;
	uint16_t node_id = 0;
	uint16_t var_id = 0;
	uint16_t var_prop = 0;
	uint16_t var_num = 0;
	uint32_t var_val = 0;
	uint16_t j = 0;
	char name[CFG_NAMECHARS];
	uint8_t name_len;

	for( uint32_t i = 0; i < CFG_IFACE_NUM; i++ )
	{
		//Transmitting
		if( (cfg_info.iface[i].tx_buf_bytes > 0) && (cfg_info.iface[i].timer == 0) )
		{
			cfg_info.iface[i].timer = CFG_TX_PERIOD;
			if( cfg_info.iface[i].tx_buf_bytes > CFG_MAX_TX_LEN )
			{
				cfg_info.iface[i].tx_func((uint8_t*)cfg_info.iface[i].tx_buf, CFG_MAX_TX_LEN);
				cfg_RemTxData(i, CFG_MAX_TX_LEN);
			}
			else
			{
				cfg_info.iface[i].tx_func((uint8_t*)cfg_info.iface[i].tx_buf, cfg_info.iface[i].tx_buf_bytes);
				cfg_info.iface[i].tx_buf_bytes = 0;
			}
		}
		//Handling received pkt
		if( cfg_info.iface[i].rvd_pkt_size > 0 )
		{
			cfg_info.iface[i].req_num += 1;
			//Answer
			tmp_u16 = CFG_HEADER;
			cfg_AddTxData(i, (uint8_t*)&tmp_u16, 2);
			cfg_AddTxData(i, (uint8_t*)&cfg_info.device_id, 2);
			cfg_AddTxData(i, (uint8_t*)&cfg_info.iface[i].rvd_pkt[5], 1); /*cmd*/
			//Handle command
			if( cfg_info.iface[i].rvd_pkt[5] == CFG_CMD_GETNODENUM )
			{
				cfg_AddTxData(i, (uint8_t*)&cfg_info.node_num, 2);
			}
			else if( cfg_info.iface[i].rvd_pkt[5] == CFG_CMD_GETNODEDESCR )
			{
				memcpy(&node_num, (uint8_t*)&cfg_info.iface[i].rvd_pkt[6], sizeof(node_num));
				if( node_num < cfg_info.node_num )
				{
					cfg_AddTxData(i, (uint8_t*)&node_num, 2);
					cfg_AddTxData(i, (uint8_t*)&cfg_NodeList[node_num].id, 2);
					cfg_AddTxData(i, (uint8_t*)&cfg_NodeList[node_num].pid, 2);
					cfg_AddTxData(i, (uint8_t*)&cfg_NodeList[node_num].var_num, 2);
					/*node name*/
					cfg_GetNodeName(cfg_NodeList[node_num].id, name);
					name_len = strlen(name) + 1;
					cfg_AddTxData(i, (uint8_t*)name, (uint32_t)name_len);
				}
				else
				{
					cfg_info.iface[i].tx_buf[4] |= 0x80;
					tmp_u8 = CFG_CMD_ERROR_NODENUM;
					cfg_AddTxData(i, (uint8_t*)&tmp_u8, 1);
				}
			}
			else if( cfg_info.iface[i].rvd_pkt[5] == CFG_CMD_GETVARDESCR )
			{
				memcpy(&node_id, (uint8_t*)&cfg_info.iface[i].rvd_pkt[6], sizeof(node_id));
				memcpy(&var_id, (uint8_t*)&cfg_info.iface[i].rvd_pkt[8], sizeof(var_id));
				if( cfg_GetNode(node_id) != 0 )
				{
					if( cfg_GetNode(node_id)->var_prop_func(var_id, name, &var_prop) == CFG_ERROR_NONE )
					{
						cfg_AddTxData(i, (uint8_t*)&node_id, 2);
						cfg_AddTxData(i, (uint8_t*)&var_id, 2);
						cfg_AddTxData(i, (uint8_t*)&var_prop, 2);
						name_len = strlen(name) + 1;
						cfg_AddTxData(i, (uint8_t*)name, (uint32_t)name_len);
					}
					else
					{
						cfg_info.iface[i].tx_buf[4] |= 0x80;
						tmp_u8 = CFG_CMD_ERROR_VARNUM;
						cfg_AddTxData(i, (uint8_t*)&tmp_u8, 1);
					}
				}
				else
				{
					cfg_info.iface[i].tx_buf[4] |= 0x80;
					tmp_u8 = CFG_CMD_ERROR_NODEID;
					cfg_AddTxData(i, (uint8_t*)&tmp_u8, 1);
				}
			}
			else if( cfg_info.iface[i].rvd_pkt[5] == CFG_CMD_GETVARS )
			{
				memcpy(&node_id, (uint8_t*)&cfg_info.iface[i].rvd_pkt[6], sizeof(node_id));
				memcpy(&var_id, (uint8_t*)&cfg_info.iface[i].rvd_pkt[8], sizeof(var_id));
				memcpy(&var_num, (uint8_t*)&cfg_info.iface[i].rvd_pkt[10], sizeof(var_num));
				if( cfg_GetNode(node_id) != 0 )
				{
					cfg_AddTxData(i, (uint8_t*)&node_id, 2);
					cfg_AddTxData(i, (uint8_t*)&var_id, 2);
					for( j = 0; j < var_num; j++ )
					{
						if( cfg_GetNode(node_id)->var_get_func(var_id + j, &var_val) == CFG_ERROR_NONE ){
							cfg_AddTxData(i, (uint8_t*)&var_val, 4);
						}
						else{
							break;
						}
					}
					cfg_IntertTxData(i, 9, (uint8_t*)&j, 2); /*insert number of parameters (variables)*/
				}
				else
				{
					cfg_info.iface[i].tx_buf[4] |= 0x80;
					tmp_u8 = CFG_CMD_ERROR_NODEID;
					cfg_AddTxData(i, (uint8_t*)&tmp_u8, 1);
				}
			}
			else if( cfg_info.iface[i].rvd_pkt[5] == CFG_CMD_SETVARS )
			{
				memcpy(&node_id, (uint8_t*)&cfg_info.iface[i].rvd_pkt[6], sizeof(node_id));
				memcpy(&var_id, (uint8_t*)&cfg_info.iface[i].rvd_pkt[8], sizeof(var_id));
				memcpy(&var_num, (uint8_t*)&cfg_info.iface[i].rvd_pkt[10], sizeof(var_num));
				if( cfg_GetNode(node_id) != 0 )
				{
					cfg_AddTxData(i, (uint8_t*)&node_id, 2);
					cfg_AddTxData(i, (uint8_t*)&var_id, 2);
					for( j = 0; j < var_num; j++ )
					{
						memcpy(&var_val, (uint8_t*)&cfg_info.iface[i].rvd_pkt[12 + (j * 4)], sizeof(var_val));
						if( (cfg_GetNode(node_id)->var_set_func(var_id + j, &var_val) == CFG_ERROR_NONE) &&
								(cfg_GetNode(node_id)->var_get_func(var_id + j, &var_val) == CFG_ERROR_NONE) ){
							cfg_AddTxData(i, (uint8_t*)&var_val, 4);
						}
						else{
							break;
						}
					}
					cfg_IntertTxData(i, 9, (uint8_t*)&j, 2); /*insert number of parameters (variables)*/
				}
				else
				{
					cfg_info.iface[i].tx_buf[4] |= 0x80;
					tmp_u8 = CFG_CMD_ERROR_NODEID;
					cfg_AddTxData(i, (uint8_t*)&tmp_u8, 1);
				}
			}
			else
			{

			}
			/*data length*/
			tmp_u8 = cfg_info.iface[i].tx_buf_bytes - 4;
			cfg_IntertTxData(i, 4, &tmp_u8, 1);
			//crc calc
			tmp_u16 = crc16_calc((uint8_t*)cfg_info.iface[i].tx_buf, cfg_info.iface[i].tx_buf_bytes);
			cfg_AddTxData(i, (uint8_t*)&tmp_u16, 2);
			cfg_info.iface[i].rvd_pkt_size = 0;
		}
	}

	return;
}

void cfg_RcvData(uint16_t iface, uint8_t data)
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	static uint16_t rcv_size;
	uint16_t header;
	uint16_t dev_id;
	uint16_t crc;

	if( iface >= CFG_IFACE_NUM ){
		return;
	}

	cfg_info.iface[iface].rx_buf.data[cfg_info.iface[iface].rx_buf.tail =
			(cfg_info.iface[iface].rx_buf.tail + 1) & CFG_BUF_MASK] = data;
	//Waiting header
	if( cfg_info.iface[iface].rx_buf.state == RINGBUF_WAIT_HEADER )
	{
		//Catch header
		header = cfg_ringBufRead16b((uint8_t*)cfg_info.iface[iface].rx_buf.data,
				(cfg_info.iface[iface].rx_buf.tail - 4) & CFG_BUF_MASK);
		dev_id = cfg_ringBufRead16b((uint8_t*)cfg_info.iface[iface].rx_buf.data,
				(cfg_info.iface[iface].rx_buf.tail - 2) & CFG_BUF_MASK);
		if( (header == CFG_HEADER) && (dev_id == cfg_info.device_id) )
		{
			cfg_info.iface[iface].rx_buf.head = (cfg_info.iface[iface].rx_buf.tail - 4) & CFG_BUF_MASK;
			rcv_size = data + 1;
			cfg_info.iface[iface].rx_buf.data_size  = data + 5;
			cfg_info.iface[iface].rx_buf.state = RINGBUF_WAIT_DATA;
		}
	}
	//Waiting data
	else if( cfg_info.iface[iface].rx_buf.state == RINGBUF_WAIT_DATA )
	{
		if( rcv_size-- != 0 ){
			return;
		}
		else
		{
			cfg_info.iface[iface].rx_buf.state = RINGBUF_WAIT_HEADER;
			//CRC check
			crc = crc16_RingBuf((uint8_t*)cfg_info.iface[iface].rx_buf.data, cfg_info.iface[iface].rx_buf.head,
					cfg_info.iface[iface].rx_buf.data_size, CFG_BUF_MASK);
			if( crc == cfg_ringBufRead16b((uint8_t*)cfg_info.iface[iface].rx_buf.data,
					(cfg_info.iface[iface].rx_buf.tail - 1) & CFG_BUF_MASK) ){
				//Copy received pkt
				if( cfg_info.iface[iface].rvd_pkt_size == 0 )
				{
					for( uint16_t i = 0; i < cfg_info.iface[iface].rx_buf.data_size; i++ ){
						cfg_info.iface[iface].rvd_pkt[i] =
							cfg_info.iface[iface].rx_buf.data[(cfg_info.iface[iface].rx_buf.head + i) & CFG_BUF_MASK];
					}
					cfg_info.iface[iface].rvd_pkt_size = cfg_info.iface[iface].rx_buf.data_size;
				}
			}
		}
	}

	return;
}

uint16_t cfg_ringBufRead16b(uint8_t *buf, uint16_t pos)
{
	uint16_t tmp;
	tmp = buf[(pos + 1) & CFG_BUF_MASK] & 0xFF;
	tmp = (tmp << 8) + buf[pos];
	return tmp;
}

uint32_t cfg_ringBufRead32b(uint8_t *buf, uint16_t pos)
{
	uint32_t tmp;
	tmp = buf[(pos + 3) & CFG_BUF_MASK] & 0xFF;
	tmp = (tmp << 8) + buf[(pos + 2) & CFG_BUF_MASK];
	tmp = (tmp << 8) + buf[(pos + 1) & CFG_BUF_MASK];
	tmp = (tmp << 8) + buf[pos];
	return tmp;
}

void cfg_SaveSettings()
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	uint16_t i, j;
	uint32_t tmp_var_buf[CFG_MAX_SAVE_PAR_NUM];
	uint16_t var_prop;
	uint16_t var_cnt = 0;
	uint32_t var_crc;

	//Count parameters to be saved
	//Nodes
	for( i = 0; i < cfg_info.node_num; i++ )
	{
		//Parameters (Variables)
		for( j = 0; j < cfg_NodeList[i].var_num; j++ )
		{
			cfg_NodeList[i].var_prop_func(j, NULL, &var_prop);
			if( (var_prop & CFG_VAR_PROP_CONST) != 0 )
			{
				cfg_NodeList[i].var_get_func(j, &tmp_var_buf[var_cnt]);
				var_cnt += 1;
			}
		}
	}
	//Writing array to flash (parameters + parameters number + crc)
	if( (var_cnt > 0) && (var_cnt < (CFG_MAX_SAVE_PAR_NUM - 2)) )
	{
		var_crc = crc32_calc(0, tmp_var_buf, var_cnt);
		HAL_FLASH_Unlock();
		//Erase sector which stores parameters array
		FLASH_Erase_Sector(FLASH_SECTOR_7, FLASH_VOLTAGE_RANGE_3);
		//Write number of parameters
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)param_arr, (uint64_t)var_cnt);
		//Write CRC
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)param_arr + 4, (uint64_t)var_crc);
		//Write parameters
		for(i = 0; i < var_cnt; i++ ){
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)param_arr + 8 + (i * 4), (uint64_t)tmp_var_buf[i]);
		}
		HAL_FLASH_Lock();

	}

	return;
}

void cfg_LoadSettings()
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	uint16_t i, j;
	uint16_t var_prop = 0;
	uint16_t var_cnt = 0;
	uint16_t load_var_cnt = 0;
	uint32_t var_num = param_arr[0]; /* Number of stored parameters */
	uint32_t var_crc = param_arr[1];

	if( (var_num > 0) && (var_num < (CFG_MAX_SAVE_PAR_NUM - 2)) )
	{
		//Count parameters to be saved
		//Nodes
		for( i = 0; i < cfg_info.node_num; i++ )
		{
			//Parameters (Variables)
			for( j = 0; j < cfg_NodeList[i].var_num; j++ )
			{
				cfg_NodeList[i].var_prop_func(j, NULL, &var_prop);
				if( (var_prop & CFG_VAR_PROP_CONST) != 0 ){
					var_cnt += 1;
				}
			}
		}

		//volatile uint32_t t_crc = crc32_calc(0, &param_arr[2], var_cnt);
		if( (var_num == var_cnt) && (var_crc == crc32_calc(0, &param_arr[2], var_cnt)) )
		{
			for( i = 0; i < cfg_info.node_num; i++ )
			{
				//Parameters (Variables)
				for( j = 0; j < cfg_NodeList[i].var_num; j++ )
				{
					cfg_NodeList[i].var_prop_func(j, NULL, &var_prop);
					if( (var_prop & CFG_VAR_PROP_CONST) != 0 )
					{
						cfg_NodeList[i].var_set_func(j, &param_arr[2 + load_var_cnt]);
						load_var_cnt += 1;
					}
				}
			}
		}
	}

	return;
}

void cfg_ClearFwBuf()
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	HAL_FLASH_Unlock();
	//todo: calc sectors from addr
	FLASH_Erase_Sector(FLASH_SECTOR_6, FLASH_VOLTAGE_RANGE_3);
	/*FLASH_Erase_Sector(FLASH_SECTOR_13, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_14, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_15, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_16, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_17, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_18, FLASH_VOLTAGE_RANGE_3);
	FLASH_Erase_Sector(FLASH_SECTOR_19, FLASH_VOLTAGE_RANGE_3);*/
	HAL_FLASH_Lock();
	cfg_new_fw_cur_addr = (uint32_t)&new_fw_start;
	cfg_new_fw_size = 0;

	return;
}

void cfg_WriteFwData(uint32_t data)
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, cfg_new_fw_cur_addr, (uint64_t)data);
	HAL_FLASH_Lock();
	cfg_new_fw_cur_addr += 4;
	cfg_new_fw_size += 4;

	return;
}

void cfg_AddTxData(uint16_t iface, uint8_t *data, uint32_t len)
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	if( iface >= CFG_IFACE_NUM ){
		return;
	}

	if( len > (CFG_BUF_SIZE - cfg_info.iface[iface].tx_buf_bytes) ){
		return;
	}
	memcpy((uint8_t*)&cfg_info.iface[iface].tx_buf[cfg_info.iface[iface].tx_buf_bytes], data, len);
	cfg_info.iface[iface].tx_buf_bytes += len;

	return;
}

void cfg_IntertTxData(uint16_t iface, uint32_t pos, uint8_t *data, uint32_t len)
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	uint8_t tmp_buf[CFG_BUF_SIZE];

	if( iface >= CFG_IFACE_NUM ){
		return;
	}
	if( len > (CFG_BUF_SIZE - cfg_info.iface[iface].tx_buf_bytes) ){
		return;
	}
	if( pos >= CFG_BUF_SIZE ){
		return;
	}
	/*tmp_buf = malloc(cfg_info.iface[iface].tx_buf_bytes - pos);
	if( tmp_buf == 0 )
	{
		tmp_buf = 1;
	}*/
	memcpy(tmp_buf, (uint8_t*)&cfg_info.iface[iface].tx_buf[pos], cfg_info.iface[iface].tx_buf_bytes - pos);
	memcpy((uint8_t*)&cfg_info.iface[iface].tx_buf[pos], data, len);
	memcpy((uint8_t*)&cfg_info.iface[iface].tx_buf[pos + len], tmp_buf, cfg_info.iface[iface].tx_buf_bytes - pos);
	//free(tmp_buf);
	cfg_info.iface[iface].tx_buf_bytes += len;

	return;
}

void cfg_RemTxData(uint16_t iface, uint32_t len)
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	uint8_t tmp_buf[CFG_BUF_SIZE];

	if( (len > cfg_info.iface[iface].tx_buf_bytes)
			|| (len == 0) || (cfg_info.iface[iface].tx_buf_bytes == 0) ){
		return;
	}
	cfg_info.iface[iface].tx_buf_bytes -= len;
	//tmp_buf = malloc(cfg_info.iface[iface].tx_buf_bytes);
	memcpy(tmp_buf, (uint8_t*)&cfg_info.iface[iface].tx_buf[len], cfg_info.iface[iface].tx_buf_bytes);
	memcpy((uint8_t*)cfg_info.iface[iface].tx_buf, tmp_buf, cfg_info.iface[iface].tx_buf_bytes);
	//free(tmp_buf);

	return;
}

void cfg_TxDataCh0(uint8_t *data, uint32_t len)
{
#ifdef NO_CFG
	return;
#endif //NO_CFG

	modem_TrmData(SYSTEM_CFG_MODEM_CH, data, len);
	//HAL_UART_Transmit(&HAL_CFG_UART, data, len, HAL_MAX_DELAY);

	return;
}
