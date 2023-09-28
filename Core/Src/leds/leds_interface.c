/**
 * @defgroup  leds_interface
 * @ref		  leds_interface
 * @author    Neat Tech
 * @brief     Software layer to handle communication between the application and the leds.
 * 			  Commands to set the leds are:
 * 			  |          Command        |                Description               |
 * 			  | :---------------------: | :--------------------------------------: |
 * 			  | l son xxx               | set led on. xxx - blue 0, red 1, green 3 |
 * 			  | l sof xxx               | set led off. xxx                         |
 * 			  | l pwm xxx yyy           | set led pwm value. xxx - led, yyy - duty |
 * 			  | l fde xxx t p           | set led fade.                            |
 * 			  | l blk xxx t0 t1 p0 p1 n | set led blink                            |
 * 			  | l fbl xxx t0 t1 p0 p1 n | set led fade blink                       |
 * 			  | l dbe                   | debug enable                             |
 * 			  | l dbd                   | debug disable                            |
 * 			  | l dmo                   | demo test                                |
 *
 * @{
*/


#include "leds_interface.h"
#include "../../config.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions

volatile leds_infoTypeDef leds_info;
volatile uint32_t leds_time_counter;


/**
  * @brief  called from SysTick_Handler to update local class time
  * @param  res - time pass in mSec
  * @retval None
  */
void leds_Timer(uint32_t res)
{
#ifdef NO_LEDS
	return;
#endif //NO_LEDS

	leds_time_counter += res;

	for( uint32_t i = 0; i < LEDS_NUM; i++ )
	{
		if( leds_info.leds[i].timer )
			leds_info.leds[i].timer--;
	}
}

/**
  * @brief  called from the main before loop
  * @param  None
  * @retval 1 if success, 0 if failed
  */
/**********************************************************************/
uint8_t leds_InitTask()
{
#ifdef NO_LEDS
	return 1;
#endif //NO_LEDS

	leds_info.leds[LED_SIM_BLUE].type = led_simple;
	leds_info.leds[LED_SIM_BLUE].sim_port = BLUE_LED_GPIO_Port;
	leds_info.leds[LED_SIM_BLUE].sim_pin = BLUE_LED_Pin;

	//Initialize beginning state
	for( uint32_t i = 0; i < LEDS_NUM; i++ )
	{
		//SIMPLE
		if( leds_info.leds[i].type == led_simple )
		{
			leds_info.leds[i].state = led_st0;
			leds_info.leds[i].mode = led_toggle;
			HAL_GPIO_WritePin(leds_info.leds[i].sim_port, leds_info.leds[i].sim_pin, GPIO_PIN_RESET);
		}
		//PWM
		else if( leds_info.leds[i].type == led_pwm )
		{
			if( leds_info.leds[i].pwmch0_tim != NULL )
			{
				leds_info.leds[i].state = led_st0;
				leds_info.leds[i].mode = led_toggle;
				leds_info.leds[i].pwmch0_value0 = LEDS_MIN_PWM_VALUE;
				leds_info.leds[i].pwmch0_cur_value = (float)LEDS_MIN_PWM_VALUE;
				if( HAL_TIM_PWM_Start(leds_info.leds[i].pwmch0_tim, leds_info.leds[i].pwmch0_timch) != HAL_OK )
				{
					return 0;
				}
				__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim, leds_info.leds[i].pwmch0_timch, leds_info.leds[i].pwmch0_value0);
			}
			else
			{
				return 0;
			}
		}
		//RGB
		else if( leds_info.leds[i].type == led_rgb )
		{
			if( (leds_info.leds[i].pwmch0_tim != NULL) && (leds_info.leds[i].pwmch1_tim != NULL) && (leds_info.leds[i].pwmch2_tim != NULL) )
			{
				leds_info.leds[i].state = led_st0;
				leds_info.leds[i].mode = led_toggle;
				leds_info.leds[i].pwmch0_value0 = LEDS_MIN_PWM_VALUE;
				leds_info.leds[i].pwmch0_cur_value = (float)LEDS_MIN_PWM_VALUE;
				leds_info.leds[i].pwmch1_value0 = LEDS_MIN_PWM_VALUE;
				leds_info.leds[i].pwmch1_cur_value = (float)LEDS_MIN_PWM_VALUE;
				leds_info.leds[i].pwmch2_value0 = LEDS_MIN_PWM_VALUE;
				leds_info.leds[i].pwmch2_cur_value = (float)LEDS_MIN_PWM_VALUE;
				if( HAL_TIM_PWM_Start(leds_info.leds[i].pwmch0_tim, leds_info.leds[i].pwmch0_timch) != HAL_OK )
				{
					return 0;
				}
				if( HAL_TIM_PWM_Start(leds_info.leds[i].pwmch1_tim, leds_info.leds[i].pwmch1_timch) != HAL_OK )
				{
					return 0;
				}
				if( HAL_TIM_PWM_Start(leds_info.leds[i].pwmch2_tim, leds_info.leds[i].pwmch2_timch) != HAL_OK )
				{
					return 0;
				}
				__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim, leds_info.leds[i].pwmch0_timch, leds_info.leds[i].pwmch0_value0);
				__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch1_tim, leds_info.leds[i].pwmch1_timch, leds_info.leds[i].pwmch1_value0);
				__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch2_tim, leds_info.leds[i].pwmch2_timch, leds_info.leds[i].pwmch2_value0);
			}
			else
			{
				return 0;
			}
		}
	}

	return 1;
}


/**
  * @brief  called from the main loop
  * @param  None
  * @retval None
  */
/**********************************************************************/
void leds_Task(void)
{
#ifdef NO_LEDS
	return;
#endif //NO_LEDS

	//Handle leds
	for( uint32_t i = 0; i < LEDS_NUM; i++ )
	{
		//Simple led
		//************************************************************************************************
		if( leds_info.leds[i].type == led_simple )
		{
			//Blink mode
			//----------------------------------------------------------------------------------------------------------------
			if( leds_info.leds[i].mode == led_blink )
			{
				//Blink state 0
				if( leds_info.leds[i].state == led_st0 )
				{
					//Wait for the timer
					if( leds_info.leds[i].timer == 0 )
					{
						//Switch state
						leds_info.leds[i].state = led_st1;
						if( leds_info.leds[i].pwmch0_value1 > 0 ) {
							HAL_GPIO_WritePin(leds_info.leds[i].sim_port, leds_info.leds[i].sim_pin, GPIO_PIN_SET);
						}
						else {
							HAL_GPIO_WritePin(leds_info.leds[i].sim_port, leds_info.leds[i].sim_pin, GPIO_PIN_RESET);
						}
						leds_info.leds[i].timer = leds_info.leds[i].anim_time_st1;
					}
				}
				//Blink state 1
				else if( leds_info.leds[i].state == led_st1 )
				{
					if( leds_info.leds[i].timer == 0 )
					{
						//Switch state
						leds_info.leds[i].state = led_st0;
						if( leds_info.leds[i].pwmch0_value0 > 0 ) {
							HAL_GPIO_WritePin(leds_info.leds[i].sim_port, leds_info.leds[i].sim_pin, GPIO_PIN_SET);
						}
						else {
							HAL_GPIO_WritePin(leds_info.leds[i].sim_port, leds_info.leds[i].sim_pin, GPIO_PIN_RESET);
						}
						//Loops
						if( (leds_info.leds[i].loop_cnt > 1) || (leds_info.leds[i].loop_cnt == 0) )
						{
							if( leds_info.leds[i].loop_cnt > 1 )
								leds_info.leds[i].loop_cnt--;
							leds_info.leds[i].timer = leds_info.leds[i].anim_time_st0;
						}
						//Stop blinking
						else
							leds_info.leds[i].mode = led_toggle;
					}
				}
			}
		}
		//PWM led
		//************************************************************************************************
		else if( leds_info.leds[i].type == led_pwm )
		{
			//Blink mode
			//----------------------------------------------------------------------------------------------------------------
			if( leds_info.leds[i].mode == led_blink )
			{
				//Blink state 0
				if( leds_info.leds[i].state == led_st0 )
				{
					//Wait for the timer
					if( leds_info.leds[i].timer == 0 )
					{
						//Switch state
						leds_info.leds[i].state = led_st1;
						leds_info.leds[i].pwmch0_cur_value = (float)leds_info.leds[i].pwmch0_value1;
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim,
												leds_info.leds[i].pwmch0_timch, leds_info.leds[i].pwmch0_value1);
						leds_info.leds[i].timer = leds_info.leds[i].anim_time_st1;
					}
				}
				//Blink state 1
				else if( leds_info.leds[i].state == led_st1 )
				{
					if( leds_info.leds[i].timer == 0 )
					{
						//Switch state
						leds_info.leds[i].state = led_st0;
						leds_info.leds[i].pwmch0_cur_value = (float)leds_info.leds[i].pwmch0_value0;
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim,
												leds_info.leds[i].pwmch0_timch, leds_info.leds[i].pwmch0_value0);
						//Loops
						if( (leds_info.leds[i].loop_cnt > 1) || (leds_info.leds[i].loop_cnt == 0) )
						{
							if( leds_info.leds[i].loop_cnt > 1 )
								leds_info.leds[i].loop_cnt--;
							leds_info.leds[i].timer = leds_info.leds[i].anim_time_st0;
						}
						//Stop blinking
						else
							leds_info.leds[i].mode = led_toggle;
					}
				}
			}
			//Fade blink mode
			//----------------------------------------------------------------------------------------------------------------
			else if( leds_info.leds[i].mode == led_fadeblink )
			{
				//
				if( leds_info.leds[i].state == led_go_to_st1 )
				{
					if( leds_info.leds[i].timer == 0 )
					{
						leds_info.leds[i].timer = LEDS_FADE_STEP_TIME;
						leds_info.leds[i].pwmch0_cur_value += leds_info.leds[i].pwmch0_step;
						if( leds_info.leds[i].step_cnt-- == 0 )
						{
							leds_info.leds[i].pwmch0_cur_value = (float)leds_info.leds[i].pwmch0_value1;
							leds_info.leds[i].state = led_go_to_st0;
							leds_info.leds[i].step_cnt = leds_info.leds[i].anim_time_st1 / LEDS_FADE_STEP_TIME;
							if( leds_info.leds[i].step_cnt == 0 )
								leds_info.leds[i].step_cnt = 1;
							leds_info.leds[i].pwmch0_step = ((float)leds_info.leds[i].pwmch0_value0 - (float)leds_info.leds[i].pwmch0_value1) / (float)leds_info.leds[i].step_cnt;
						}
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim,
											leds_info.leds[i].pwmch0_timch, (uint8_t)leds_info.leds[i].pwmch0_cur_value);
					}
				}
				else if( leds_info.leds[i].state == led_go_to_st0 )
				{
					if( leds_info.leds[i].timer == 0 )
					{
						leds_info.leds[i].timer = LEDS_FADE_STEP_TIME;
						leds_info.leds[i].pwmch0_cur_value += leds_info.leds[i].pwmch0_step;
						if( leds_info.leds[i].step_cnt-- == 0 )
						{
							if( (leds_info.leds[i].loop_cnt > 1) || (leds_info.leds[i].loop_cnt == 0) )
							{
								if( leds_info.leds[i].loop_cnt > 1 )
									leds_info.leds[i].loop_cnt--;
								leds_info.leds[i].pwmch0_cur_value = (float)leds_info.leds[i].pwmch0_value0;
								leds_info.leds[i].state = led_go_to_st1;
								leds_info.leds[i].step_cnt = leds_info.leds[i].anim_time_st0 / LEDS_FADE_STEP_TIME;
								if( leds_info.leds[i].step_cnt == 0 )
									leds_info.leds[i].step_cnt = 1;
								leds_info.leds[i].pwmch0_step = ((float)leds_info.leds[i].pwmch0_value1 - (float)leds_info.leds[i].pwmch0_value0) / (float)leds_info.leds[i].step_cnt;
							}
							else
								//leds_info.leds[i].mode = led_toggle;
								leds_setFade(i, 200, LEDS_MIN_PWM_VALUE);
						}
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim,
											leds_info.leds[i].pwmch0_timch, (uint8_t)leds_info.leds[i].pwmch0_cur_value);
					}
				}
			}
			//Fade mode
			//----------------------------------------------------------------------------------------------------------------
			else if( leds_info.leds[i].mode == led_fade )
			{
				if( leds_info.leds[i].timer == 0 )
				{
					leds_info.leds[i].timer = LEDS_FADE_STEP_TIME;
					leds_info.leds[i].pwmch0_cur_value += leds_info.leds[i].pwmch0_step;
					if( leds_info.leds[i].step_cnt-- == 0 )
					{
						leds_info.leds[i].pwmch0_cur_value = (float)leds_info.leds[i].pwmch0_value1;
						leds_info.leds[i].mode = led_toggle;
					}
					__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim,
										leds_info.leds[i].pwmch0_timch, (uint8_t)leds_info.leds[i].pwmch0_cur_value);
				}
			}
		}
		//RGB led
		//************************************************************************************************
		else if( leds_info.leds[i].type == led_rgb )
		{
			//Blink mode
			//----------------------------------------------------------------------------------------------------------------
			if( leds_info.leds[i].mode == led_blink )
			{
				//Blink state 0
				if( leds_info.leds[i].state == led_st0 )
				{
					//Wait for the timer
					if( leds_info.leds[i].timer == 0 )
					{
						//Switch state
						leds_info.leds[i].state = led_st1;
						leds_info.leds[i].pwmch0_cur_value = (float)leds_info.leds[i].pwmch0_value1;
						leds_info.leds[i].pwmch1_cur_value = (float)leds_info.leds[i].pwmch1_value1;
						leds_info.leds[i].pwmch2_cur_value = (float)leds_info.leds[i].pwmch2_value1;
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim,
												leds_info.leds[i].pwmch0_timch, leds_info.leds[i].pwmch0_value1);
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch1_tim,
												leds_info.leds[i].pwmch1_timch, leds_info.leds[i].pwmch1_value1);
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch2_tim,
												leds_info.leds[i].pwmch2_timch, leds_info.leds[i].pwmch2_value1);
						leds_info.leds[i].timer = leds_info.leds[i].anim_time_st1;

					}
				}
				//Blink state 1
				else if( leds_info.leds[i].state == led_st1 )
				{
					if( leds_info.leds[i].timer == 0 )
					{
						//Switch state
						leds_info.leds[i].state = led_st0;
						leds_info.leds[i].pwmch0_cur_value = (float)leds_info.leds[i].pwmch0_value0;
						leds_info.leds[i].pwmch1_cur_value = (float)leds_info.leds[i].pwmch1_value0;
						leds_info.leds[i].pwmch2_cur_value = (float)leds_info.leds[i].pwmch2_value0;
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim,
												leds_info.leds[i].pwmch0_timch, leds_info.leds[i].pwmch0_value0);
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch1_tim,
												leds_info.leds[i].pwmch1_timch, leds_info.leds[i].pwmch1_value0);
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch2_tim,
												leds_info.leds[i].pwmch2_timch, leds_info.leds[i].pwmch2_value0);
						//Loops
						if( (leds_info.leds[i].loop_cnt > 1) || (leds_info.leds[i].loop_cnt == 0) )
						{
							if( leds_info.leds[i].loop_cnt > 1 )
								leds_info.leds[i].loop_cnt--;
							leds_info.leds[i].timer = leds_info.leds[i].anim_time_st0;
						}
						//Stop blinking
						else
							leds_info.leds[i].mode = led_toggle;
					}
				}
			}
			//Fade blink mode
			//----------------------------------------------------------------------------------------------------------------
			else if( leds_info.leds[i].mode == led_fadeblink )
			{
				//
				if( leds_info.leds[i].state == led_go_to_st1 )
				{
					if( leds_info.leds[i].timer == 0 )
					{
						leds_info.leds[i].timer = LEDS_FADE_STEP_TIME;
						leds_info.leds[i].pwmch0_cur_value += leds_info.leds[i].pwmch0_step;
						leds_info.leds[i].pwmch1_cur_value += leds_info.leds[i].pwmch1_step;
						leds_info.leds[i].pwmch2_cur_value += leds_info.leds[i].pwmch2_step;
						if( leds_info.leds[i].step_cnt-- == 0 )
						{
							leds_info.leds[i].pwmch0_cur_value = (float)leds_info.leds[i].pwmch0_value1;
							leds_info.leds[i].pwmch1_cur_value = (float)leds_info.leds[i].pwmch1_value1;
							leds_info.leds[i].pwmch2_cur_value = (float)leds_info.leds[i].pwmch2_value1;
							leds_info.leds[i].state = led_go_to_st0;
							leds_info.leds[i].step_cnt = leds_info.leds[i].anim_time_st1 / LEDS_FADE_STEP_TIME;
							if( leds_info.leds[i].step_cnt == 0 )
								leds_info.leds[i].step_cnt = 1;
							leds_info.leds[i].pwmch0_step = ((float)leds_info.leds[i].pwmch0_value0 - (float)leds_info.leds[i].pwmch0_value1) / (float)leds_info.leds[i].step_cnt;
							leds_info.leds[i].pwmch1_step = ((float)leds_info.leds[i].pwmch1_value0 - (float)leds_info.leds[i].pwmch1_value1) / (float)leds_info.leds[i].step_cnt;
							leds_info.leds[i].pwmch2_step = ((float)leds_info.leds[i].pwmch2_value0 - (float)leds_info.leds[i].pwmch2_value1) / (float)leds_info.leds[i].step_cnt;
						}
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim,
											leds_info.leds[i].pwmch0_timch, (uint8_t)leds_info.leds[i].pwmch0_cur_value);
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch1_tim,
											leds_info.leds[i].pwmch1_timch, (uint8_t)leds_info.leds[i].pwmch1_cur_value);
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch2_tim,
											leds_info.leds[i].pwmch2_timch, (uint8_t)leds_info.leds[i].pwmch2_cur_value);
					}
				}
				else if( leds_info.leds[i].state == led_go_to_st0 )
				{
					if( leds_info.leds[i].timer == 0 )
					{
						leds_info.leds[i].timer = LEDS_FADE_STEP_TIME;
						leds_info.leds[i].pwmch0_cur_value += leds_info.leds[i].pwmch0_step;
						leds_info.leds[i].pwmch1_cur_value += leds_info.leds[i].pwmch1_step;
						leds_info.leds[i].pwmch2_cur_value += leds_info.leds[i].pwmch2_step;
						if( leds_info.leds[i].step_cnt-- == 0 )
						{
							if( (leds_info.leds[i].loop_cnt > 1) || (leds_info.leds[i].loop_cnt == 0) )
							{
								if( leds_info.leds[i].loop_cnt > 1 )
									leds_info.leds[i].loop_cnt--;
								leds_info.leds[i].pwmch0_cur_value = (float)leds_info.leds[i].pwmch0_value0;
								leds_info.leds[i].pwmch1_cur_value = (float)leds_info.leds[i].pwmch1_value0;
								leds_info.leds[i].pwmch2_cur_value = (float)leds_info.leds[i].pwmch2_value0;
								leds_info.leds[i].state = led_go_to_st1;
								leds_info.leds[i].step_cnt = leds_info.leds[i].anim_time_st0 / LEDS_FADE_STEP_TIME;
								if( leds_info.leds[i].step_cnt == 0 )
									leds_info.leds[i].step_cnt = 1;
								leds_info.leds[i].pwmch0_step = ((float)leds_info.leds[i].pwmch0_value1 - (float)leds_info.leds[i].pwmch0_value0) / (float)leds_info.leds[i].step_cnt;
								leds_info.leds[i].pwmch1_step = ((float)leds_info.leds[i].pwmch1_value1 - (float)leds_info.leds[i].pwmch1_value0) / (float)leds_info.leds[i].step_cnt;
								leds_info.leds[i].pwmch2_step = ((float)leds_info.leds[i].pwmch2_value1 - (float)leds_info.leds[i].pwmch2_value0) / (float)leds_info.leds[i].step_cnt;
							}
							else
								//leds_info.leds[i].mode = led_toggle;
								leds_setFade(i, 200, LEDS_MIN_PWM_VALUE);
						}
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim,
											leds_info.leds[i].pwmch0_timch, (uint8_t)leds_info.leds[i].pwmch0_cur_value);
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch1_tim,
											leds_info.leds[i].pwmch1_timch, (uint8_t)leds_info.leds[i].pwmch1_cur_value);
						__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch2_tim,
											leds_info.leds[i].pwmch2_timch, (uint8_t)leds_info.leds[i].pwmch2_cur_value);
					}
				}
			}
			//Fade mode
			//----------------------------------------------------------------------------------------------------------------
			else if( leds_info.leds[i].mode == led_fade )
			{
				if( leds_info.leds[i].timer == 0 )
				{
					leds_info.leds[i].timer = LEDS_FADE_STEP_TIME;
					leds_info.leds[i].pwmch0_cur_value += leds_info.leds[i].pwmch0_step;
					leds_info.leds[i].pwmch1_cur_value += leds_info.leds[i].pwmch1_step;
					leds_info.leds[i].pwmch2_cur_value += leds_info.leds[i].pwmch2_step;
					if( leds_info.leds[i].step_cnt-- == 0 )
					{
						leds_info.leds[i].pwmch0_cur_value = (float)leds_info.leds[i].pwmch0_value1;
						leds_info.leds[i].pwmch1_cur_value = (float)leds_info.leds[i].pwmch1_value1;
						leds_info.leds[i].pwmch2_cur_value = (float)leds_info.leds[i].pwmch2_value1;
						leds_info.leds[i].mode = led_toggle;
					}
					__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch0_tim,
										leds_info.leds[i].pwmch0_timch, (uint8_t)leds_info.leds[i].pwmch0_cur_value);
					__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch1_tim,
										leds_info.leds[i].pwmch1_timch, (uint8_t)leds_info.leds[i].pwmch1_cur_value);
					__HAL_TIM_SET_COMPARE(leds_info.leds[i].pwmch2_tim,
										leds_info.leds[i].pwmch2_timch, (uint8_t)leds_info.leds[i].pwmch2_cur_value);
				}
			}
		}
	}

	return;
}

/**
  * @brief  set led to on state
  * @param  s_led - value from leds_sys enum
  * @retval 1 if success, 0 if failed
  */
uint8_t leds_setOn(uint32_t s_led)
{
#ifdef NO_LEDS
	return 1;
#endif //NO_LEDS

	if( s_led >= LEDS_NUM )
	{
		return 0;
	}

	//SIMPLE
	if( leds_info.leds[s_led].type == led_simple )
	{
		leds_info.leds[s_led].state = led_st1;
		leds_info.leds[s_led].mode = led_toggle;
		HAL_GPIO_WritePin(leds_info.leds[s_led].sim_port, leds_info.leds[s_led].sim_pin, GPIO_PIN_SET);
	}
	//PWM led
	else if( leds_info.leds[s_led].type == led_pwm )
	{
		leds_info.leds[s_led].state = led_st1;
		leds_info.leds[s_led].mode = led_toggle;
		leds_info.leds[s_led].pwmch0_cur_value = (float)LEDS_MAX_PWM_VALUE;
		leds_info.leds[s_led].pwmch0_value1 = LEDS_MAX_PWM_VALUE;
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value1);
	}
	//RGB
	else if( leds_info.leds[s_led].type == led_rgb )
	{
		leds_info.leds[s_led].state = led_st1;
		leds_info.leds[s_led].mode = led_toggle;
		leds_info.leds[s_led].pwmch0_cur_value = (float)LEDS_MAX_PWM_VALUE;
		leds_info.leds[s_led].pwmch0_value1 = LEDS_MAX_PWM_VALUE;
		leds_info.leds[s_led].pwmch1_cur_value = (float)LEDS_MAX_PWM_VALUE;
		leds_info.leds[s_led].pwmch1_value1 = LEDS_MAX_PWM_VALUE;
		leds_info.leds[s_led].pwmch2_cur_value = (float)LEDS_MAX_PWM_VALUE;
		leds_info.leds[s_led].pwmch2_value1 = LEDS_MIN_PWM_VALUE;
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value1);
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch1_tim, leds_info.leds[s_led].pwmch1_timch, leds_info.leds[s_led].pwmch1_value1);
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch2_tim, leds_info.leds[s_led].pwmch2_timch, leds_info.leds[s_led].pwmch2_value1);
	}

	return 1;
}

/**
  * @brief  set led to off state
  * @param  s_led - value from leds_sys enum
  * @retval 1 if success, 0 if failed
  */
uint8_t leds_setOff(uint32_t s_led)
{
#ifdef NO_LEDS
	return 1;
#endif //NO_LEDS

	if( s_led >= LEDS_NUM )
	{
		return 0;
	}

	//SIMPLE
	if( leds_info.leds[s_led].type == led_simple )
	{
		leds_info.leds[s_led].state = led_st0;
		leds_info.leds[s_led].mode = led_toggle;
		HAL_GPIO_WritePin(leds_info.leds[s_led].sim_port, leds_info.leds[s_led].sim_pin, GPIO_PIN_RESET);
	}
	//PWM led
	else if( leds_info.leds[s_led].type == led_pwm )
	{
		leds_info.leds[s_led].state = led_st0;
		leds_info.leds[s_led].mode = led_toggle;
		leds_info.leds[s_led].pwmch0_cur_value = (float)LEDS_MIN_PWM_VALUE;
		leds_info.leds[s_led].pwmch0_value0 = LEDS_MIN_PWM_VALUE;
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value0);
	}
	//RGB
	else if( leds_info.leds[s_led].type == led_rgb )
	{
		leds_info.leds[s_led].state = led_st0;
		leds_info.leds[s_led].mode = led_toggle;
		leds_info.leds[s_led].pwmch0_cur_value = (float)LEDS_MIN_PWM_VALUE;
		leds_info.leds[s_led].pwmch0_value0 = LEDS_MIN_PWM_VALUE;
		leds_info.leds[s_led].pwmch1_cur_value = (float)LEDS_MIN_PWM_VALUE;
		leds_info.leds[s_led].pwmch1_value0 = LEDS_MIN_PWM_VALUE;
		leds_info.leds[s_led].pwmch2_cur_value = (float)LEDS_MIN_PWM_VALUE;
		leds_info.leds[s_led].pwmch2_value0 = LEDS_MIN_PWM_VALUE;
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value0);
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch1_tim, leds_info.leds[s_led].pwmch1_timch, leds_info.leds[s_led].pwmch1_value0);
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch2_tim, leds_info.leds[s_led].pwmch2_timch, leds_info.leds[s_led].pwmch2_value0);
	}

	return 1;
}

/**
  * @brief  set pwm value for the led
  * @param  s_led 	- value from leds_sys enum
  * @param  pwm_val - pwm value can be for pwm led from LEDS_MIN_PWM_VALUE to LEDS_MAX_PWM_VALUE
  * 					or 0xXXBBGGRR format for RGB led
  * @retval 1 if success, 0 if failed
  */
uint8_t leds_setPwm(uint32_t s_led, uint32_t pwm_val)
{
#ifdef NO_LEDS
	return 1;
#endif //NO_LEDS

	if( s_led >= LEDS_NUM )
	{
		return 0;
	}

	//PWM
	if( leds_info.leds[s_led].type == led_pwm )
	{
		if( (pwm_val < LEDS_MIN_PWM_VALUE) || (pwm_val > LEDS_MAX_PWM_VALUE) )
		{
			return 0;
		}
		leds_info.leds[s_led].state = led_st0;
		leds_info.leds[s_led].mode = led_toggle;
		leds_info.leds[s_led].pwmch0_cur_value = (float)pwm_val;
		leds_info.leds[s_led].pwmch0_value0 = (uint8_t)pwm_val;
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value0);

	}
	//RGB
	else if( leds_info.leds[s_led].type == led_rgb )
	{
		leds_info.leds[s_led].state = led_st0;
		leds_info.leds[s_led].mode = led_toggle;
		leds_info.leds[s_led].pwmch0_cur_value = (float)(pwm_val & 0xFF);
		leds_info.leds[s_led].pwmch0_value0 = (uint8_t)(pwm_val & 0xFF);
		leds_info.leds[s_led].pwmch1_cur_value = (float)((pwm_val >> 8) & 0xFF);
		leds_info.leds[s_led].pwmch1_value0 = (uint8_t)((pwm_val >> 8) & 0xFF);
		leds_info.leds[s_led].pwmch2_cur_value = (float)((pwm_val >> 16) & 0xFF);
		leds_info.leds[s_led].pwmch2_value0 = (uint8_t)((pwm_val >> 16) & 0xFF);
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value0);
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch1_tim, leds_info.leds[s_led].pwmch1_timch, leds_info.leds[s_led].pwmch1_value0);
		__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch2_tim, leds_info.leds[s_led].pwmch2_timch, leds_info.leds[s_led].pwmch2_value0);

	}
	else
	{
		return 0;
	}



	return 1;
}

/**
  * @brief  set PWM led to blink mode
  * @param  s_led 	- value from leds_sys enum
  * @param  st0_time - duration of state 0 in ms
  * @param  st1_time - duration of state 1 in ms
  * @param  loops 	- number of repeating (0 for infinite)
  * @param  val0 	- pwm value can be for pwm led from LEDS_MIN_PWM_VALUE to LEDS_MAX_PWM_VALUE
  * 					or 0xXXBBGGRR format for RGB led
  * @param  val1 	- pwm value can be for pwm led from LEDS_MIN_PWM_VALUE to LEDS_MAX_PWM_VALUE
  * 					or 0xXXBBGGRR format for RGB led
  * @retval 1 if success, 0 if failed
  */
uint8_t leds_setBlink(uint32_t s_led, uint32_t st0_time, uint32_t st1_time, uint32_t loops, uint32_t val0, uint32_t val1)
{
#ifdef NO_LEDS
	return 1;
#endif //NO_LEDS

	if( s_led >= LEDS_NUM )
	{
		return 0;
	}

	//SIMPLE
	if( leds_info.leds[s_led].type == led_simple )
	{
		leds_info.leds[s_led].pwmch0_value0 = val0;
		leds_info.leds[s_led].pwmch0_value1 = val1;
		leds_info.leds[s_led].loop_cnt = loops;
		leds_info.leds[s_led].anim_time_st0 = st0_time;
		leds_info.leds[s_led].anim_time_st1 = st1_time;
		if( leds_info.leds[s_led].mode != led_blink )
		{
			leds_info.leds[s_led].state = led_st0;
			leds_info.leds[s_led].mode = led_blink;
			leds_info.leds[s_led].timer = leds_info.leds[s_led].anim_time_st0;
			if( leds_info.leds[s_led].pwmch0_value0 > 0 ) {
				HAL_GPIO_WritePin(leds_info.leds[s_led].sim_port, leds_info.leds[s_led].sim_pin, GPIO_PIN_SET);
			}
			else {
				HAL_GPIO_WritePin(leds_info.leds[s_led].sim_port, leds_info.leds[s_led].sim_pin, GPIO_PIN_RESET);
			}
		}
	}
	//PWM
	else if( leds_info.leds[s_led].type == led_pwm )
	{
		if( (val0 < LEDS_MIN_PWM_VALUE) || (val0 > LEDS_MAX_PWM_VALUE) || (val1 < LEDS_MIN_PWM_VALUE) || (val1 > LEDS_MAX_PWM_VALUE) )
		{
			return 0;
		}
		leds_info.leds[s_led].pwmch0_value0 = val0;
		leds_info.leds[s_led].pwmch0_value1 = val1;
		leds_info.leds[s_led].loop_cnt = loops;
		leds_info.leds[s_led].anim_time_st0 = st0_time;
		leds_info.leds[s_led].anim_time_st1 = st1_time;
		if( leds_info.leds[s_led].mode != led_blink )
		{
			leds_info.leds[s_led].state = led_st0;
			leds_info.leds[s_led].mode = led_blink;
			leds_info.leds[s_led].timer = leds_info.leds[s_led].anim_time_st0;
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value0);
		}
	}
	//RGB
	else if( leds_info.leds[s_led].type == led_rgb )
	{
		leds_info.leds[s_led].pwmch0_value0 = (uint8_t)(val0 & 0xFF);
		leds_info.leds[s_led].pwmch1_value0 = (uint8_t)((val0 >> 8) & 0xFF);
		leds_info.leds[s_led].pwmch2_value0 = (uint8_t)((val0 >> 16) & 0xFF);
		leds_info.leds[s_led].pwmch0_value1 = (uint8_t)(val1 & 0xFF);
		leds_info.leds[s_led].pwmch1_value1 = (uint8_t)((val1 >> 8) & 0xFF);
		leds_info.leds[s_led].pwmch2_value1 = (uint8_t)((val1 >> 16) & 0xFF);
		leds_info.leds[s_led].loop_cnt = loops;
		leds_info.leds[s_led].anim_time_st0 = st0_time;
		leds_info.leds[s_led].anim_time_st1 = st1_time;
		if( leds_info.leds[s_led].mode != led_blink )
		{
			leds_info.leds[s_led].state = led_st0;
			leds_info.leds[s_led].mode = led_blink;
			leds_info.leds[s_led].timer = leds_info.leds[s_led].anim_time_st0;
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value0);
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch1_tim, leds_info.leds[s_led].pwmch1_timch, leds_info.leds[s_led].pwmch1_value0);
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch2_tim, leds_info.leds[s_led].pwmch2_timch, leds_info.leds[s_led].pwmch2_value0);
		}
	}

	return 1;
}

/**
  * @brief  set fade to pwm value value from corrent pwm value
  * @param  s_led 	- value from leds_sys enum
  * @param  fade_time - duration of faiding in ms
  * @param  pwm_val - pwm value can be for pwm led from LEDS_MIN_PWM_VALUE to LEDS_MAX_PWM_VALUE
  * 					or 0xXXBBGGRR format for RGB led
  * @retval 1 if success, 0 if failed
  */
uint8_t leds_setFade(uint32_t s_led, uint32_t fade_time, uint32_t pwm_val)
{
#ifdef NO_LEDS
	return 1;
#endif //NO_LEDS

	if( s_led >= LEDS_NUM )
	{
		return 0;
	}

	//PWM led
	if( leds_info.leds[s_led].type == led_pwm )
	{
		if( (pwm_val < LEDS_MIN_PWM_VALUE) || (pwm_val > LEDS_MAX_PWM_VALUE) )
		{
			return 0;
		}
		leds_info.leds[s_led].pwmch0_value0 = (uint8_t)leds_info.leds[s_led].pwmch0_cur_value;
		leds_info.leds[s_led].pwmch0_value1 = (uint8_t)pwm_val;
		leds_info.leds[s_led].anim_time_st0 = fade_time;
		if( leds_info.leds[s_led].mode != led_fade )
		{
			leds_info.leds[s_led].step_cnt = fade_time / LEDS_FADE_STEP_TIME;
			if( leds_info.leds[s_led].step_cnt == 0 )
				leds_info.leds[s_led].step_cnt = 1;
			leds_info.leds[s_led].pwmch0_step = ((float)pwm_val - (float)leds_info.leds[s_led].pwmch0_value0) / (float)leds_info.leds[s_led].step_cnt;
			leds_info.leds[s_led].mode = led_fade;
			leds_info.leds[s_led].timer = LEDS_FADE_STEP_TIME;
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value0);
		}
	}
	//RGB
	else if( leds_info.leds[s_led].type == led_rgb )
	{
		leds_info.leds[s_led].pwmch0_value0 = (uint8_t)leds_info.leds[s_led].pwmch0_cur_value;
		leds_info.leds[s_led].pwmch0_value1 = (uint8_t)(pwm_val & 0xFF);
		leds_info.leds[s_led].pwmch1_value0 = (uint8_t)leds_info.leds[s_led].pwmch1_cur_value;
		leds_info.leds[s_led].pwmch1_value1 = (uint8_t)((pwm_val >> 8) & 0xFF);
		leds_info.leds[s_led].pwmch2_value0 = (uint8_t)leds_info.leds[s_led].pwmch2_cur_value;
		leds_info.leds[s_led].pwmch2_value1 = (uint8_t)((pwm_val >> 16) & 0xFF);;
		leds_info.leds[s_led].anim_time_st0 = fade_time;
		if( leds_info.leds[s_led].mode != led_fade )
		{
			leds_info.leds[s_led].step_cnt = fade_time / LEDS_FADE_STEP_TIME;
			if( leds_info.leds[s_led].step_cnt == 0 )
				leds_info.leds[s_led].step_cnt = 1;
			leds_info.leds[s_led].pwmch0_step = ((float)leds_info.leds[s_led].pwmch0_value1 - (float)leds_info.leds[s_led].pwmch0_value0) / (float)leds_info.leds[s_led].step_cnt;
			leds_info.leds[s_led].pwmch1_step = ((float)leds_info.leds[s_led].pwmch1_value1 - (float)leds_info.leds[s_led].pwmch1_value0) / (float)leds_info.leds[s_led].step_cnt;
			leds_info.leds[s_led].pwmch2_step = ((float)leds_info.leds[s_led].pwmch2_value1 - (float)leds_info.leds[s_led].pwmch2_value0) / (float)leds_info.leds[s_led].step_cnt;
			leds_info.leds[s_led].mode = led_fade;
			leds_info.leds[s_led].timer = LEDS_FADE_STEP_TIME;
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value0);
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch1_tim, leds_info.leds[s_led].pwmch1_timch, leds_info.leds[s_led].pwmch1_value0);
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch2_tim, leds_info.leds[s_led].pwmch2_timch, leds_info.leds[s_led].pwmch2_value0);
		}
	}
	else
	{
		return 0;
	}

	return 1;
}

/**
  * @brief  set PWM led to fade blink mode
  * @param  s_led 	- value from leds_sys enum
  * @param  st0_time - duration of state 0 in ms
  * @param  st1_time - duration of state 1 in ms
  * @param  loops 	- number of repeating (0 for infinite)
  * @param  val0 - pwm value can be for pwm led from LEDS_MIN_PWM_VALUE to LEDS_MAX_PWM_VALUE
  * 					or 0xXXBBGGRR format for RGB led
  * @param  val1 - pwm value can be for pwm led from LEDS_MIN_PWM_VALUE to LEDS_MAX_PWM_VALUE
  * 					or 0xXXBBGGRR format for RGB led
  * @retval 1 if success, 0 if failed
  */
uint8_t leds_setFadeBlink(uint32_t s_led, uint32_t st0_time, uint32_t st1_time, uint32_t loops, uint32_t val0, uint32_t val1)
{
#ifdef NO_LEDS
	return 1;
#endif //NO_LEDS

	if( s_led >= LEDS_NUM )
	{
		return 0;
	}

	//PWM led
	if( leds_info.leds[s_led].type == led_pwm )
	{
		if( (val0 < LEDS_MIN_PWM_VALUE) || (val0 > LEDS_MAX_PWM_VALUE) || (val1 < LEDS_MIN_PWM_VALUE) || (val1 > LEDS_MAX_PWM_VALUE) )
		{
			return 0;
		}
		leds_info.leds[s_led].pwmch0_value0 = val0;
		leds_info.leds[s_led].pwmch0_value1 = val1;
		leds_info.leds[s_led].pwmch0_cur_value = (float)leds_info.leds[s_led].pwmch0_value0;
		leds_info.leds[s_led].loop_cnt = loops;
		leds_info.leds[s_led].anim_time_st0 = st0_time;
		leds_info.leds[s_led].anim_time_st1 = st1_time;
		if( leds_info.leds[s_led].mode != led_fadeblink )
		{
			leds_info.leds[s_led].step_cnt = st0_time / LEDS_FADE_STEP_TIME;
			if( leds_info.leds[s_led].step_cnt == 0 )
				leds_info.leds[s_led].step_cnt = 1;
			leds_info.leds[s_led].pwmch0_step = ((float)val1 - (float)val0) / (float)leds_info.leds[s_led].step_cnt;
			leds_info.leds[s_led].state = led_go_to_st1;
			leds_info.leds[s_led].mode = led_fadeblink;
			leds_info.leds[s_led].timer = LEDS_FADE_STEP_TIME;
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value0);
		}
	}
	else if( leds_info.leds[s_led].type == led_rgb )
	{
		leds_info.leds[s_led].pwmch0_value0 = (uint8_t)(val0 & 0xFF);
		leds_info.leds[s_led].pwmch0_value1 = (uint8_t)(val1 & 0xFF);
		leds_info.leds[s_led].pwmch1_value0 = (uint8_t)((val0 >> 8) & 0xFF);
		leds_info.leds[s_led].pwmch1_value1 = (uint8_t)((val1 >> 8) & 0xFF);
		leds_info.leds[s_led].pwmch2_value0 = (uint8_t)((val0 >> 16) & 0xFF);
		leds_info.leds[s_led].pwmch2_value1 = (uint8_t)((val1 >> 16) & 0xFF);
		leds_info.leds[s_led].pwmch0_cur_value = (float)leds_info.leds[s_led].pwmch0_value0;
		leds_info.leds[s_led].pwmch1_cur_value = (float)leds_info.leds[s_led].pwmch1_value0;
		leds_info.leds[s_led].pwmch2_cur_value = (float)leds_info.leds[s_led].pwmch2_value0;
		leds_info.leds[s_led].loop_cnt = loops;
		leds_info.leds[s_led].anim_time_st0 = st0_time;
		leds_info.leds[s_led].anim_time_st1 = st1_time;
		if( leds_info.leds[s_led].mode != led_fadeblink )
		{
			leds_info.leds[s_led].step_cnt = st0_time / LEDS_FADE_STEP_TIME;
			if( leds_info.leds[s_led].step_cnt == 0 )
				leds_info.leds[s_led].step_cnt = 1;
			leds_info.leds[s_led].pwmch0_step = ((float)leds_info.leds[s_led].pwmch0_value1 - (float)leds_info.leds[s_led].pwmch0_value0) / (float)leds_info.leds[s_led].step_cnt;
			leds_info.leds[s_led].pwmch1_step = ((float)leds_info.leds[s_led].pwmch1_value1 - (float)leds_info.leds[s_led].pwmch1_value0) / (float)leds_info.leds[s_led].step_cnt;
			leds_info.leds[s_led].pwmch2_step = ((float)leds_info.leds[s_led].pwmch2_value1 - (float)leds_info.leds[s_led].pwmch2_value0) / (float)leds_info.leds[s_led].step_cnt;
			leds_info.leds[s_led].state = led_go_to_st1;
			leds_info.leds[s_led].mode = led_fadeblink;
			leds_info.leds[s_led].timer = LEDS_FADE_STEP_TIME;
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch0_tim, leds_info.leds[s_led].pwmch0_timch, leds_info.leds[s_led].pwmch0_value0);
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch1_tim, leds_info.leds[s_led].pwmch1_timch, leds_info.leds[s_led].pwmch1_value0);
			__HAL_TIM_SET_COMPARE(leds_info.leds[s_led].pwmch2_tim, leds_info.leds[s_led].pwmch2_timch, leds_info.leds[s_led].pwmch2_value0);
		}
}
	else
	{
		return 0;
	}

	return 1;
}
