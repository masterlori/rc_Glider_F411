/**
 ******************************************************************************
 * @file      ledss_interface.h
 * @author    Neat Tech
 * @brief     Header leds interface module
 *
 *            Software layer to handle leds.
 *
 ******************************************************************************/


#ifndef SRC_LEDS_INTERFACE_H_
#define SRC_LEDS_INTERFACE_H_

#include "../inc/main.h"
#include <stdio.h>
#include <inttypes.h>

#define LEDS_MIN_PWM_VALUE			0
#define LEDS_MAX_PWM_VALUE			255
#define LEDS_FADE_STEP_TIME			20

enum leds_sys
{
	LED_SIM_BLUE,
	LEDS_NUM
};

typedef enum
{
	led_simple,
	led_pwm,
	led_rgb
} ledtype_TypeDef;

typedef enum
{
	led_toggle,
	led_blink,
	led_fadeblink,
	led_fade
} ledmode_TypeDef;

typedef enum
{
	led_st0,
	led_st1,
	led_go_to_st0,
	led_go_to_st1
} ledstate_TypeDef;

typedef struct __led_TypeDef
{
	ledtype_TypeDef type;
	ledstate_TypeDef state;
	ledmode_TypeDef mode;
	uint32_t timer;
	float pwmch0_cur_value;
	uint8_t pwmch0_value0;
	uint8_t pwmch0_value1;
	float pwmch0_step;
	float pwmch1_cur_value;
	uint8_t pwmch1_value0;
	uint8_t pwmch1_value1;
	float pwmch1_step;
	float pwmch2_cur_value;
	uint8_t pwmch2_value0;
	uint8_t pwmch2_value1;
	float pwmch2_step;
	uint32_t anim_time_st0;
	uint32_t anim_time_st1;
	uint32_t loop_cnt;
	uint32_t step_cnt;
	GPIO_TypeDef* sim_port;
	uint16_t      sim_pin;
	TIM_HandleTypeDef *pwmch0_tim;
	uint32_t pwmch0_timch;
	TIM_HandleTypeDef *pwmch1_tim;
	uint32_t pwmch1_timch;
	TIM_HandleTypeDef *pwmch2_tim;
	uint32_t pwmch2_timch;
	GPIO_TypeDef toggle_gpio;
	uint16_t toggle_pin;
} led_TypeDef;

typedef struct __leds_infoTypeDef{
	led_TypeDef leds[LEDS_NUM];
	uint8_t error_flag;
	uint32_t error_line;
	uint8_t debug_enabled;
}leds_infoTypeDef;

extern volatile leds_infoTypeDef leds_info;
extern void leds_Timer(uint32_t res);
extern void leds_Task(void);
extern uint8_t leds_InitTask(void);
extern uint8_t leds_setOn(uint32_t s_led);
extern uint8_t leds_setOff(uint32_t s_led);
extern uint8_t leds_setPwm(uint32_t s_led, uint32_t pwm_val);
extern uint8_t leds_setFade(uint32_t s_led, uint32_t fade_time, uint32_t pwm_val);
extern uint8_t leds_setBlink(uint32_t s_led, uint32_t st0_time,
							uint32_t st1_time, uint32_t loops, uint32_t val0, uint32_t val1);
extern uint8_t leds_setFadeBlink(uint32_t s_led, uint32_t st0_time,
							uint32_t st1_time, uint32_t loops, uint32_t val0, uint32_t val1);


#endif /* SRC_LEDS_INTERFACE_H_ */
