
#include "main.h"
#include "motor_interface.h"
#include "../configurator/cfg_interface.h"

volatile motor_infoTypeDef motor_info;

int32_t motor_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

void motor_Timer(uint32_t res)
{
#ifdef NO_MOTOR
	return;
#endif //NO_MOTOR

	for( uint8_t i = 0; i < MOTOR_TMR_NUM; i++ )
	{
		if( motor_info.timer[i] > 0 ){
			motor_info.timer[i]--;
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
void motor_InitTask()
{
#ifdef NO_MOTOR
	return;
#endif //NO_MOTOR

	motor_info.motors[MOTOR_MAIN].tim = &htim3;
	motor_info.motors[MOTOR_MAIN].timch = TIM_CHANNEL_3;
	motor_info.motors[MOTOR_MAIN].cur_torq = 0;
	motor_info.motors[MOTOR_MAIN].min_pwm = 535;
	motor_info.motors[MOTOR_MAIN].idle_pwm = 426;
	motor_info.motors[MOTOR_MAIN].cur_pwm = motor_info.motors[MOTOR_MAIN].idle_pwm;
	motor_info.motors[MOTOR_MAIN].max_pwm = 950;
	motor_info.motors[MOTOR_MAIN].enable = 1;

	//Initialize beginning state
	for( uint8_t i = 0; i < MOTORS_NUM; i++ )
	{
		HAL_TIM_PWM_Start(motor_info.motors[i].tim, motor_info.motors[i].timch);
	}

	motor_setTorque(MOTOR_MAIN, motor_info.motors[MOTOR_MAIN].cur_torq);

	return;
}

/**
  * @brief  called from the main loop
  * @param  None
  * @retval None
  */
/**********************************************************************/
void motor_Task(void)
{
#ifdef NO_MOTOR
	return;
#endif //NO_MOTOR

	return;
}

void motor_setEnable(uint8_t motor, uint8_t ena)
{
#ifdef NO_MOTOR
	return;
#endif //NO_MOTOR

	if( motor >= MOTORS_NUM ){
		return;
	}

	motor_info.motors[motor].enable = ena;

	if( motor_info.motors[motor].enable == 1 ){
		__HAL_TIM_SET_COMPARE(motor_info.motors[motor].tim, motor_info.motors[motor].timch, motor_info.motors[motor].cur_pwm);
	}
	else{
		__HAL_TIM_SET_COMPARE(motor_info.motors[motor].tim, motor_info.motors[motor].timch, motor_info.motors[motor].idle_pwm);
	}

	return;
}

void motor_setTorque(uint8_t motor, uint8_t torq)
{
#ifdef NO_MOTOR
	return;
#endif //NO_MOTOR

	if( motor >= MOTORS_NUM ){
		return;
	}

	motor_info.motors[motor].cur_torq = torq;
	if( torq == 0 ){
		motor_info.motors[motor].cur_pwm = motor_info.motors[motor].idle_pwm;
	}
	else
	{
		motor_info.motors[motor].cur_pwm =
				motor_map((int32_t)motor_info.motors[motor].cur_torq,
							1, 100,
							(int32_t)motor_info.motors[motor].min_pwm,
							(int32_t)motor_info.motors[motor].max_pwm);
	}
	if( motor_info.motors[motor].enable == 1 ){
		__HAL_TIM_SET_COMPARE(motor_info.motors[motor].tim, motor_info.motors[motor].timch, motor_info.motors[motor].cur_pwm);
	}

	return;
}

/**
  * @brief  Re-maps a number from one range to another. That is, a value of fromLow would get mapped to toLow,
  * 		a value of fromHigh to toHigh,
  * 		values in-between to values in-between, etc.
  * @param  value: the number to map.
  * 	fromLow: the lower bound of the value’s current range.
  * 	fromHigh: the upper bound of the value’s current range.
  * 	toLow: the lower bound of the value’s target range.
  * 	toHigh: the upper bound of the value’s target range.
  * @retval The mapped value.
  */
int32_t motor_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Configurator node functions*/
/*************************************************************************/
uint16_t cfg_NodeMotorVarProp(uint16_t varid, char *name, uint16_t *prop)
{
	char *str;

	switch( varid )
	{
		case MOTOR_ENA:				str = "Enable"; break;
		case MOTOR_CUR_TORQ:		str = "Current torque"; break;
		case MOTOR_CUR_PWM:			str = "Current PWM"; break;
		case MOTOR_IDLE_PWM:		str = "Idle PWM"; break;
		case MOTOR_MIN_PWM:			str = "Min PWM"; break;
		case MOTOR_MAX_PWM:			str = "Max PWM"; break;
		default: return CFG_ERROR_VARID;
	}
	if( name ) { while( *str ) *name++ = *str++; *name = 0; }

	if( prop ) switch( varid )
	{
		case MOTOR_ENA:				*prop = CFG_VAR_TYPE_BOOL | CFG_VAR_PROP_CONST; break;
		case MOTOR_CUR_TORQ:		*prop = CFG_VAR_TYPE_UINT; break;
		case MOTOR_CUR_PWM:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_READONLY; break;
		case MOTOR_IDLE_PWM:		*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_CONST; break;
		case MOTOR_MIN_PWM:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_CONST; break;
		case MOTOR_MAX_PWM:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_CONST; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeMotorVarGet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case MOTOR_ENA: 				*(uint32_t*)value = (uint32_t)motor_info.motors[MOTOR_MAIN].enable; break;
		case MOTOR_CUR_TORQ: 			*(uint32_t*)value = (uint32_t)motor_info.motors[MOTOR_MAIN].cur_torq; break;
		case MOTOR_CUR_PWM: 			*(uint32_t*)value = (uint32_t)motor_info.motors[MOTOR_MAIN].cur_pwm; break;
		case MOTOR_IDLE_PWM: 			*(uint32_t*)value = (int32_t)motor_info.motors[MOTOR_MAIN].idle_pwm; break;
		case MOTOR_MIN_PWM: 			*(uint32_t*)value = (int32_t)motor_info.motors[MOTOR_MAIN].min_pwm; break;
		case MOTOR_MAX_PWM: 			*(uint32_t*)value = (uint32_t)motor_info.motors[MOTOR_MAIN].max_pwm; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeMotorVarSet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case MOTOR_ENA:				motor_setEnable(MOTOR_MAIN, (uint8_t)*(uint32_t*)value); break;
		case MOTOR_CUR_TORQ:		motor_setTorque(MOTOR_MAIN, (uint8_t)*(uint32_t*)value); break;
		case MOTOR_CUR_PWM:			break;
		case MOTOR_IDLE_PWM:		motor_info.motors[MOTOR_MAIN].idle_pwm = (uint16_t)*(uint32_t*)value; break;
		case MOTOR_MIN_PWM:			motor_info.motors[MOTOR_MAIN].min_pwm = (uint16_t)*(uint32_t*)value; break;
		case MOTOR_MAX_PWM:			motor_info.motors[MOTOR_MAIN].max_pwm = (uint16_t)*(uint32_t*)value; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}


