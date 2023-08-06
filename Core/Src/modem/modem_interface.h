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

#ifndef SRC_MODEM_MODEM_INTERFACE_H_
#define SRC_MODEM_MODEM_INTERFACE_H_

#define MODEM_HEADER		0xAA55
#define MODEM_MAX_TX_LEN 	20
#define MODEM_TX_PERIOD 	1
#define MODEM_FW_VAR_NUM 	48
#define MODEM_BUF_SIZE 		128
#define MODEM_BUF_MASK (MODEM_BUF_SIZE - 1)
#if    (MODEM_BUF_SIZE & MODEM_BUF_MASK)
#error  MODEM_BUF_SIZE is not a power of 2
#endif

enum modem_timers
{
	MODEM_TX_TMR,
	MODEM_STATE_TMR,
	MODEM_LED_TMR,
	MODEM_TMR_NUM
};

typedef enum
{
	MDMRINGBUF_WAIT_HEADER,
	MDMRINGBUF_WAIT_DATA
} modemringbufstate_TypeDef;

typedef struct __modem_ringbufTypeDef
{
	modemringbufstate_TypeDef state;
	uint16_t head;
	uint16_t tail;
	uint16_t data_size;
	uint8_t data[MODEM_BUF_SIZE];
}modem_ringbufTypeDef;

typedef struct __modem_infoTypeDef{
	uint32_t timer[MODEM_TMR_NUM];
	modem_ringbufTypeDef rx_buf;
	uint8_t rvd_pkt[MODEM_BUF_SIZE];
	uint32_t rvd_pkt_size;
	uint8_t tx_buf[MODEM_BUF_SIZE];
	uint32_t tx_buf_bytes;
	uint8_t error_flag;
	uint32_t error_line;
	uint8_t debug_enabled;
	uint8_t ready_flag;
}modem_infoTypeDef;

extern volatile modem_infoTypeDef modem_info;
extern void modem_Timer(uint32_t res);
extern void modem_InitTask(void);
extern void modem_Task(void);
extern void modem_RcvData(uint8_t data);
uint16_t modem_ringBufRead16b(uint8_t *buf, uint16_t pos);
uint32_t modem_ringBufRead32b(uint8_t *buf, uint16_t pos);
extern void modem_AddTxData(uint8_t *data, uint32_t len);
extern void modem_RemTxData(uint32_t len);
extern void modem_TrmData(uint8_t channel, uint8_t *data, uint32_t len);

#endif /* SRC_MODEM_MODEM_INTERFACE_H_ */
