

#ifndef SRC_RC_RC_INTERFACE_H_
#define SRC_RC_RC_INTERFACE_H_

#define RC_TLM_SND_PERIOD	1000
#define RC_DEF_CONN_TOUT	3000
#define RC_HEADER			0x8CC8
#define RC_BUF_SIZE 		128
#define RC_BUF_MASK (RC_BUF_SIZE - 1)
#if    (RC_BUF_SIZE & RC_BUF_MASK)
#error  RC_BUF_SIZE is not a power of 2
#endif


enum rc_timers
{
	RC_TMR_CONN,
	RC_TMR_TRM_TLM,
	RC_TMR_NUM
};

typedef enum
{
	RCRINGBUF_WAIT_HEADER,
	RCRINGBUF_WAIT_DATA
} rcringbufstate_TypeDef;

typedef struct __rc_ringbufTypeDef
{
	rcringbufstate_TypeDef state;
	uint16_t head;
	uint16_t tail;
	uint16_t data_size;
	uint8_t data[RC_BUF_SIZE];
}rc_ringbufTypeDef;

typedef struct __rc_infoTypeDef{
	uint32_t timer[RC_TMR_NUM];
	rc_ringbufTypeDef rx_buf;
	uint8_t rvd_pkt[RC_BUF_SIZE];
	uint32_t rvd_pkt_size;
	uint32_t conn_tout;
	uint8_t connected;
	uint8_t enable;
	uint8_t mode;
	uint8_t send_tlm;
	uint32_t rx_pkt_cnt;
	int8_t	axis_l_x;
	int8_t	axis_l_y;
	int8_t	axis_r_x;
	int8_t	axis_r_y;
	uint8_t button_l;
	uint8_t button_r;
	uint8_t error_flag;
	uint32_t error_line;
	uint8_t debug_enabled;
}rc_infoTypeDef;

extern volatile rc_infoTypeDef rc_info;
extern void rc_Timer(uint32_t res);
extern void rc_InitTask(void);
extern void rc_Task(void);
extern void rc_RcvData(uint8_t data);

//Configurator parameters
enum
{
	RC_ENA,
	RC_MODE,
	RC_SEND_TLM,
	RC_RX_PKT_CNT,
	RC_AXIS_L_X,
	RC_AXIS_L_Y,
	RC_AXIS_R_X,
	RC_AXIS_R_Y,
	RC_BUT_L,
	RC_BUT_R,
	RC_VAR_NUM,
};

extern uint16_t cfg_NodeRcVarProp(uint16_t varid, char *name, uint16_t *prop);
extern uint16_t cfg_NodeRcVarGet(uint16_t varid, void *value);
extern uint16_t cfg_NodeRcVarSet(uint16_t varid, void *value);

#endif /* SRC_RC_RC_INTERFACE_H_ */
