/**
 ******************************************************************************
 * @file      system.h
 * @author    Neat Tech
 * @brief     Header system
 *
 *            system file handle the entire device flow, coordinate between interfaces.
 *
 ******************************************************************************/


#ifndef SRC_SYSTEM_H_
#define SRC_SYSTEM_H_

#include "../inc/main.h"

#define SYSTEM_CFG_MODEM_CH	1
#define SYSTEM_RC_MODEM_CH	2

/* @brief software timers for the stimulation module */
enum
{
	sys_tmr_reset,
	sys_tmr_adc,
	sys_tmr_num
};

//State of the EEPROM interface
typedef enum
{
	system_ready,
	system_init

} systemstate_TypeDef;

typedef struct __system_infoTypeDef{
	systemstate_TypeDef state;
	uint8_t reset_req;
	uint8_t adc_cplt;
	uint32_t timer[sys_tmr_num];
}system_infoTypeDef;

extern volatile system_infoTypeDef system_info;
extern void system_Init(void);
extern void system_Task(void);
extern void system_Timer(uint32_t res);
extern void system_UARTmodemSet115200();
extern void system_UART_RxCpltCallback(UART_HandleTypeDef *huart);
extern void system_ModemRxCallback(uint8_t channel, uint8_t *data, uint32_t len);
extern void system_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
extern __ramfunc void system_Reset();
#endif /* SRC_SYSTEM_H_ */
