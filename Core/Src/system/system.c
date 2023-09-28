/**
 * @defgroup  system_interface
 * @ref		  system
 * @author    Neat Tech
 * @brief     system handle the entire device flow, coordinate between interfaces. \n
 *            This interface contain all call backs (adc/dma/interrupt/timers) \n
 *            and all the tasks are being called from here. \n
 *
 * @{
*/


#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions

#include "system.h"
#include "main.h"
#include "../../config.h"
#include "../leds/leds_interface.h"
#include "../modem/modem_interface.h"
#include "../configurator/cfg_interface.h"
#include "../motor/motor_interface.h"
#include "../servo/servo_interface.h"
#include "../rc/rc_interface.h"
#include "../autopilot/autopilot.h"
#include "../battery/battery_interface.h"
#include "../sensors/sens_interface.h"
#include "usbd_cdc_if.h"


volatile system_infoTypeDef system_info;
volatile uint32_t system_time_counter;
volatile uint8_t cfg_uart_rx_data;
volatile uint8_t _system_restart_uart = 0;

void system_ErrorHandler();

/**
  * @brief  called from SysTick_Handler to update local class time
  * @param  res - time pass in mSec
  * @retval None
  */
void system_Timer(uint32_t res)
{
	system_time_counter += res;
	leds_Timer(res);
	cfg_Timer(res);
	modem_Timer(res);
	servo_Timer(res);
	motor_Timer(res);
	rc_Timer(res);
	battery_Timer(res);
	autopilot_Timer(res);
	sens_Timer(res);

	for( uint8_t i = 0; i < sys_tmr_num; i++ )
	{
		if( system_info.timer[i] != 0 ){
			system_info.timer[i]--;
		}
	}
}


/**
  * @brief  Full system init after power up
  * @param  None
  * @retval None
  */
void system_Init()
{
	system_info.reset_req = 0;
	leds_InitTask();
	modem_InitTask();
	servo_InitTask();
	motor_InitTask();
	rc_InitTask();
	sens_InitTask();
	battery_InitTask(10000);
	autopilot_InitTask();
	cfg_InitTask();
	system_info.usb_rx_flag = 0;
	HAL_UART_Receive_IT(&HAL_MODEM_UART, (uint8_t*)&cfg_uart_rx_data, 1);
	HAL_ADC_Start_IT(&hadc1);
	//leds_setBlink(LED_SIM_BLUE, 200, 1000, 0, 0, 1);
}

/**
  * @brief  Full system flow, runs in while loop from main.c file
  * @param  None
  * @retval None
  */
void system_Task(void)
{
	uint32_t i;

	leds_Task();
	cfg_Task();
	modem_Task();
	motor_Task();
	servo_Task();
	rc_Task();
	sens_Task();
	battery_Task();
	autopilot_Task();

	//ADC
	if( (system_info.adc_cplt == 1) && (system_info.timer[sys_tmr_adc] == 0) )
	{
		system_info.adc_cplt = 0;
		 HAL_ADC_Start_IT(&hadc1);
	}

	//UART restart
	if( _system_restart_uart == 1 )
	{
		_system_restart_uart = 0;
		MX_UARTReInit();
		if( HAL_UART_Receive_IT(&HAL_MODEM_UART, (uint8_t*)&cfg_uart_rx_data, 1) != HAL_OK ){
			_system_restart_uart = 1;
			//system_ErrorHandler();
		}
	}

	//USB Rx
	if( system_info.usb_rx_flag == 1 )
	{
		for( i = 0; i < system_info.usb_rx_len; i++ ){
			cfg_RcvData(CFG_IFACE_CH1_USB, system_info.usb_rx_buf[i]);
		}
		system_info.usb_rx_flag = 0;
	}

	//System reset
	if( system_info.reset_req == 1 )
	{
		system_info.reset_req = 2;
		system_info.timer[sys_tmr_reset] = 500;
	}
	if( (system_info.reset_req == 2) && (system_info.timer[sys_tmr_reset] == 0) )
	{
		system_info.reset_req = 0;
		system_Reset();
	}
}

void system_UARTmodemSet115200()
{
	HAL_UART_Abort_IT(&HAL_MODEM_UART);
	HAL_UART_DeInit(&HAL_MODEM_UART);
	MX_USART1_UART_Init115200();
	HAL_UART_Receive_IT(&HAL_MODEM_UART, (uint8_t*)&cfg_uart_rx_data, 1);
	return;
}
void system_ModemRxCallback(uint8_t channel, uint8_t *data, uint32_t len)
{
	uint32_t i;

	if( channel == SYSTEM_CFG_MODEM_CH )
	{
		for( i = 0; i < len; i++ ){
			cfg_RcvData(CFG_IFACE_CH0, data[i]);
		}
	}
	else if( channel == SYSTEM_RC_MODEM_CH )
	{
		for( i = 0; i < len; i++ ){
			rc_RcvData(data[i]);
		}
	}


	return;
}

void system_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if( hadc == &hadc1){
		battery_info.battery_volatge = (uint16_t)((float)HAL_ADC_GetValue(&hadc1) *
										((float)3300 / (float)4095) * (float)BATTERY_MEAS_COEF);
		battery_info.battery_volatge_updated_flag = 1;
		system_info.adc_cplt = 1;
		system_info.timer[sys_tmr_adc] = 1000;
	}

	return;
}

/**
  * @brief RX interrupt from the uart
  * @param None
  * @retval None
  */
void system_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart == &HAL_MODEM_UART )
	{
		//cfg_RcvData(CFG_IFACE_CH0, cfg_uart_rx_data);
		modem_RcvData(cfg_uart_rx_data);
		if( HAL_UART_Receive_IT(&HAL_MODEM_UART, (uint8_t*)&cfg_uart_rx_data, 1) != HAL_OK ){
			//system_ErrorHandler();
			_system_restart_uart = 1;
		}
	}
}

__ramfunc void system_Reset()
{
	__disable_irq();
	SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
	SCB_AIRCR_SYSRESETREQ_Msk); 																/* Keep priority group unchanged *//* Ensure completion of memory access */
	__DSB();                                                                             		// Ensure completion of memory access
	while(1);
}

void system_ErrorHandler()
{
	while(1){

	}
}

/** @} */
