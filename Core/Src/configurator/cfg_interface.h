/**
 ******************************************************************************
 * @file      leds_interface.h
 * @author    Neat Tech
 * @brief     Header leds interface module
 *
 *            Software layer to handle leds.
 *
 ******************************************************************************/

#include "main.h"

#ifndef SRC_CONFIGURATOR_CFG_INTERFACE_H_
#define SRC_CONFIGURATOR_CFG_INTERFACE_H_

#define CFG_HEADER 		0xCEFA
#define CFG_MAX_TX_LEN 	64
#define CFG_TX_PERIOD 	0
#define CFG_FW_VAR_NUM 	48
#define CFG_BUF_SIZE 	256
#define CFG_BUF_MASK (CFG_BUF_SIZE - 1)
#if    (CFG_BUF_SIZE & CFG_BUF_MASK)
#error  CFG_BUF_SIZE is not a power of 2
#endif

/* @brief configurator protocol commands */
#define CFG_MAX_SAVE_PAR_NUM	1024

/* @brief configurator protocol commands */
//Get number of nodes
#define CFG_CMD_GETNODENUM		0x01
//Get nodes properties
#define CFG_CMD_GETNODEDESCR	0x02
//Get parameters properties
#define CFG_CMD_GETVARDESCR		0x03
//Get parameters value
#define CFG_CMD_GETVARS			0x10
//Set parameters value
#define CFG_CMD_SETVARS			0x11

/* @brief configurator protocol errors */
//No errors
#define CFG_CMD_ERROR_COMMAND	1
//Wrong node id
#define CFG_CMD_ERROR_NODENUM	2
//Wrong node id
#define CFG_CMD_ERROR_NODEID	3
//Wrong parameter (variable) id
#define CFG_CMD_ERROR_VARNUM	4
//Wrong parameter (variable) value
#define CFG_CMD_ERROR_VARVAL	5
//Wrong parameter (variable) value
#define CFG_CMD_ERROR_UNKNOWN	6

/* @brief configuration errors */
//No errors
#define CFG_ERROR_NONE			0
//Wrong node id
#define CFG_ERROR_NODEID		1
//Wrong parameter (variable) id
#define CFG_ERROR_VARID			2
//Wrong parameter (variable) value
#define CFG_ERROR_VARVAL		3

/* @brief configurator parameters (variables) types */
#define CFG_VAR_TYPE_INT		0x01
#define CFG_VAR_TYPE_UINT		0x02
#define CFG_VAR_TYPE_BOOL		0x03
#define CFG_VAR_TYPE_REAL		0x04
#define CFG_VAR_TYPE_HEX		0x05
#define CFG_VAR_TYPE_CHAR4		0x06
#define CFG_VAR_TYPE_TIME		0x07
#define CFG_VAR_TYPE_MASK       0x0F

/* @brief configurator parameters (variables) properties */
//Parameter is read only
#define CFG_VAR_PROP_READONLY	0x10
//Parameter stores in memory
#define CFG_VAR_PROP_CONST		0x20
//Parameter is error code
#define CFG_VAR_PROP_ERROR		0x40
//Parameter records to the "Black box"
#define CFG_VAR_PROP_TRACE		0x80
#define CFG_VAR_PROP_MASK		0xF0

/* @brief maximum length of parameters name */
#define CFG_NAMECHARS			64

/* @brief firmware node parameters num */
#define CFG_FIRMWARE_VAR_NUM	48

/* @brief configurator node */
typedef struct __cgf_nodeTypeDef
{
	uint16_t id; 		/*ID of the node*/
	uint16_t pid; 		/*ID of the parent node*/
	uint16_t var_num; 	/*Number of parameters (variables)*/
	uint16_t (*var_get_func) (uint16_t varid, void *value);
	uint16_t (*var_set_func) (uint16_t varid, void *value);
	uint16_t (*var_prop_func) (uint16_t varid, char *name, uint16_t *prop);
}cgf_nodeTypeDef;

/* @brief list of nodes */
extern cgf_nodeTypeDef cfg_NodeList[];

/* @brief get the name of the node */
extern uint16_t cfg_GetNodeName(uint16_t nodeid, char *name);

/* @brief get node*/
extern cgf_nodeTypeDef* cfg_GetNode(uint16_t nodeid);

enum cfg_ifaces
{
	CFG_IFACE_CH0,
	CFG_IFACE_CH1_USB,
	CFG_IFACE_NUM
};

typedef enum
{
	RINGBUF_WAIT_HEADER,
	RINGBUF_WAIT_DATA
} ringbufstate_TypeDef;

typedef struct __cgf_ringbufTypeDef
{
	ringbufstate_TypeDef state;
	uint16_t head;
	uint16_t tail;
	uint16_t data_size;
	uint8_t data[CFG_BUF_SIZE];
}cgf_ringbufTypeDef;

typedef struct __cfg_ifaceTypeDef
{
	cgf_ringbufTypeDef rx_buf;
	uint8_t tx_buf[CFG_BUF_SIZE];
	uint32_t tx_buf_bytes;
	void (*tx_func) (uint8_t *data, uint32_t len);
	uint32_t timer;
	uint8_t rvd_pkt[CFG_BUF_SIZE];
	uint32_t rvd_pkt_size;
	uint32_t req_num;
} cfg_ifaceTypeDef;

typedef struct __cfg_infoTypeDef{
	cfg_ifaceTypeDef iface[CFG_IFACE_NUM];
	uint16_t device_id;
	uint32_t par_0;
	uint16_t node_num;
	uint8_t error_flag;
	uint32_t error_line;
	uint8_t debug_enabled;
	uint8_t new_fw_received;
}cfg_infoTypeDef;

extern volatile cfg_infoTypeDef cfg_info;
extern void cfg_Timer(uint32_t res);
extern void cfg_Task(void);
extern void cfg_InitTask(void);
uint16_t cfg_ringBufRead16b(uint8_t *buf, uint16_t pos);
uint32_t cfg_ringBufRead32b(uint8_t *buf, uint16_t pos);
extern void cfg_RcvData(uint16_t iface, uint8_t data);
extern void cfg_AddTxData(uint16_t iface, uint8_t *data, uint32_t len);
extern void cfg_IntertTxData(uint16_t iface, uint32_t pos, uint8_t *data, uint32_t len);
extern void cfg_RemTxData(uint16_t iface, uint32_t len);
extern void cfg_SaveSettings();
extern void cfg_LoadSettings();
extern void cfg_ClearFwBuf();
extern void cfg_WriteFwData(uint32_t data);

#endif /* SRC_CONFIGURATOR_CFG_INTERFACE_H_ */
