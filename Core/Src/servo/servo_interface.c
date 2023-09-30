
#include "main.h"
#include "servo_interface.h"
#include "../configurator/cfg_interface.h"

volatile servo_infoTypeDef servo_info;

uint16_t servo_angToPWM(uint16_t ang);
int32_t servo_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

void servo_Timer(uint32_t res)
{
#ifdef NO_SERVO
	return;
#endif //NO_SERVO

	for( uint8_t i = 0; i < SERVO_TMR_NUM; i++ )
	{
		if( servo_info.timer[i] > 0 ){
			servo_info.timer[i]--;
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
void servo_InitTask()
{
#ifdef NO_SERVO
	return;
#endif //NO_SERVO

	servo_info.servos[SERVO_ROLL].tim = &htim3;
	servo_info.servos[SERVO_ROLL].timch = TIM_CHANNEL_1;
	servo_info.servos[SERVO_ROLL].cur_ang = 0;
	servo_info.servos[SERVO_ROLL].cur_pwm = 0;
	servo_info.servos[SERVO_ROLL].cur_perc = 0;
	servo_info.servos[SERVO_ROLL].min_ang = 50;
	servo_info.servos[SERVO_ROLL].center_ang = 105;
	servo_info.servos[SERVO_ROLL].max_ang = 145;
	servo_info.servos[SERVO_ROLL].enable = 1;

	servo_info.servos[SERVO_PITCH].tim = &htim3;
	servo_info.servos[SERVO_PITCH].timch = TIM_CHANNEL_2;
	servo_info.servos[SERVO_PITCH].cur_ang = 0;
	servo_info.servos[SERVO_PITCH].cur_pwm = 0;
	servo_info.servos[SERVO_PITCH].cur_perc = 0;
	servo_info.servos[SERVO_PITCH].min_ang = 25;
	servo_info.servos[SERVO_PITCH].center_ang = 90;
	servo_info.servos[SERVO_PITCH].max_ang = 140;
	servo_info.servos[SERVO_PITCH].enable = 1;

	//Initialize beginning state
	for( uint8_t i = 0; i < SERVOS_NUM; i++ )
	{
		servo_info.servos[i].cur_pwm = 0;
		servo_info.servos[i].cur_ang = 0;
		HAL_TIM_PWM_Start(servo_info.servos[i].tim, servo_info.servos[i].timch);
	}

	servo_setAng(SERVO_ROLL, servo_info.servos[SERVO_ROLL].center_ang);
	servo_setAng(SERVO_PITCH, servo_info.servos[SERVO_PITCH].center_ang);

	return;
}

/**
  * @brief  called from the main loop
  * @param  None
  * @retval None
  */
/**********************************************************************/
void servo_Task(void)
{
#ifdef NO_SERVO
	return;
#endif //NO_SERVO

	return;
}

uint16_t servo_angToPWM(uint16_t ang)
{
	uint16_t pwm_val;

	pwm_val = SERVO_0DEG_PWM_VAL + (ang * SERVO_1DEG_PWM_STEP);

	return pwm_val;
}

void servo_setAng(uint8_t servo, uint16_t ang)
{
#ifdef NO_SERVO
	return;
#endif //NO_SERVO

	if( servo >= SERVOS_NUM ){
		return;
	}

	servo_info.servos[servo].cur_ang = ang;
	servo_info.servos[servo].cur_pwm = servo_angToPWM(ang);
	if( servo_info.servos[servo].enable == 1 ){
		__HAL_TIM_SET_COMPARE(servo_info.servos[servo].tim, servo_info.servos[servo].timch, servo_info.servos[servo].cur_pwm);
	}

	return;
}

void servo_setPercnet(uint8_t servo, int8_t perc)
{
#ifdef NO_SERVO
	return;
#endif //NO_SERVO

	uint16_t ang_tmp;

	if( servo >= SERVOS_NUM ){
		return;
	}

	if( (perc > 100) || (perc < -100) ){
		return;
	}

	if( perc >= 0 )
	{
		ang_tmp = (uint16_t)servo_map((int32_t)perc, 0, 100,
										(int32_t)servo_info.servos[servo].center_ang,
										(int32_t)servo_info.servos[servo].max_ang);
	}
	else
	{
		ang_tmp = (uint16_t)servo_map((int32_t)perc, -1, -100,
										(int32_t)servo_info.servos[servo].center_ang,
										(int32_t)servo_info.servos[servo].min_ang);
	}

	servo_setAng(servo, ang_tmp);

	return;
}

void servo_setEnable(uint8_t servo, uint8_t ena)
{
#ifdef NO_SERVO
	return;
#endif //NO_SERVO

	if( servo >= SERVOS_NUM ){
		return;
	}

	servo_info.servos[servo].enable = ena;

	if( servo_info.servos[servo].enable == 1 ){
		__HAL_TIM_SET_COMPARE(servo_info.servos[servo].tim, servo_info.servos[servo].timch, servo_info.servos[servo].cur_pwm);
	}
	else{
		__HAL_TIM_SET_COMPARE(servo_info.servos[servo].tim, servo_info.servos[servo].timch, 0);
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
int32_t servo_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Configurator node functions*/
/*************************************************************************/
uint16_t cfg_NodeServoVarProp(uint16_t varid, char *name, uint16_t *prop)
{
	char *str;

	switch( varid )
	{
		case SERVO_ROLL_ENA:				str = "ROLL enable"; break;
		case SERVO_ROLL_CUR_ANG:			str = "ROLL current angle"; break;
		case SERVO_ROLL_CUR_PWM:			str = "ROLL current PWM"; break;
		case SERVO_ROLL_CUR_PERC:			str = "ROLL current perc"; break;
		case SERVO_ROLL_MIN_ANG:			str = "ROLL min angle"; break;
		case SERVO_ROLL_CENT_ANG:			str = "ROLL center angle"; break;
		case SERVO_ROLL_MAX_ANG:			str = "ROLL max angle"; break;
		case SERVO_PITCH_ENA:				str = "PITCH enable"; break;
		case SERVO_PITCH_CUR_ANG:			str = "PITCH current angle"; break;
		case SERVO_PITCH_CUR_PWM:			str = "PITCH current PWM"; break;
		case SERVO_PITCH_CUR_PERC:			str = "PITCH current perc"; break;
		case SERVO_PITCH_MIN_ANG:			str = "PITCH min angle"; break;
		case SERVO_PITCH_CENT_ANG:			str = "PITCH center angle"; break;
		case SERVO_PITCH_MAX_ANG:			str = "PITCH max angle"; break;
		default: return CFG_ERROR_VARID;
	}
	if( name ) { while( *str ) *name++ = *str++; *name = 0; }

	if( prop ) switch( varid )
	{
		case SERVO_ROLL_ENA:				*prop = CFG_VAR_TYPE_BOOL | CFG_VAR_PROP_CONST; break;
		case SERVO_ROLL_CUR_ANG:			*prop = CFG_VAR_TYPE_UINT; break;
		case SERVO_ROLL_CUR_PWM:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_READONLY; break;
		case SERVO_ROLL_CUR_PERC:			*prop = CFG_VAR_TYPE_INT; break;
		case SERVO_ROLL_MIN_ANG:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_CONST; break;
		case SERVO_ROLL_CENT_ANG:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_CONST; break;
		case SERVO_ROLL_MAX_ANG:	  		*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_CONST; break;
		case SERVO_PITCH_ENA:				*prop = CFG_VAR_TYPE_BOOL | CFG_VAR_PROP_CONST; break;
		case SERVO_PITCH_CUR_ANG:			*prop = CFG_VAR_TYPE_UINT; break;
		case SERVO_PITCH_CUR_PWM:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_READONLY; break;
		case SERVO_PITCH_CUR_PERC:			*prop = CFG_VAR_TYPE_INT; break;
		case SERVO_PITCH_MIN_ANG:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_CONST; break;
		case SERVO_PITCH_CENT_ANG:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_CONST; break;
		case SERVO_PITCH_MAX_ANG:	  		*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_CONST; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeServoVarGet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case SERVO_ROLL_ENA: 				*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_ROLL].enable; break;
		case SERVO_ROLL_CUR_ANG: 			*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_ROLL].cur_ang; break;
		case SERVO_ROLL_CUR_PWM: 			*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_ROLL].cur_pwm; break;
		case SERVO_ROLL_CUR_PERC: 			*(uint32_t*)value = (int32_t)servo_info.servos[SERVO_ROLL].cur_perc; break;
		case SERVO_ROLL_MIN_ANG: 			*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_ROLL].min_ang; break;
		case SERVO_ROLL_CENT_ANG: 			*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_ROLL].center_ang; break;
		case SERVO_ROLL_MAX_ANG: 			*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_ROLL].max_ang; break;
		case SERVO_PITCH_ENA: 				*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_PITCH].enable; break;
		case SERVO_PITCH_CUR_ANG: 			*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_PITCH].cur_ang; break;
		case SERVO_PITCH_CUR_PWM: 			*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_PITCH].cur_pwm; break;
		case SERVO_PITCH_CUR_PERC: 			*(uint32_t*)value = (int32_t)servo_info.servos[SERVO_PITCH].cur_perc; break;
		case SERVO_PITCH_MIN_ANG: 			*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_PITCH].min_ang; break;
		case SERVO_PITCH_CENT_ANG: 			*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_PITCH].center_ang; break;
		case SERVO_PITCH_MAX_ANG: 			*(uint32_t*)value = (uint32_t)servo_info.servos[SERVO_PITCH].max_ang; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeServoVarSet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case SERVO_ROLL_ENA:				servo_setEnable(SERVO_ROLL, (uint8_t)*(uint32_t*)value); break;
		case SERVO_ROLL_CUR_ANG:			servo_setAng(SERVO_ROLL, (uint16_t)*(uint32_t*)value); break;
		case SERVO_ROLL_CUR_PWM:			break;
		case SERVO_ROLL_CUR_PERC:			servo_setPercnet(SERVO_ROLL, (int8_t)*(uint32_t*)value); break;
		case SERVO_ROLL_MIN_ANG:			servo_info.servos[SERVO_ROLL].min_ang = *(uint32_t*)value; break;
		case SERVO_ROLL_CENT_ANG:			servo_info.servos[SERVO_ROLL].center_ang = *(uint32_t*)value; break;
		case SERVO_ROLL_MAX_ANG:			servo_info.servos[SERVO_ROLL].max_ang = *(uint32_t*)value; break;
		case SERVO_PITCH_ENA:				servo_setEnable(SERVO_PITCH, (uint8_t)*(uint32_t*)value); break;
		case SERVO_PITCH_CUR_ANG:			servo_setAng(SERVO_PITCH, (uint16_t)*(uint32_t*)value); break;
		case SERVO_PITCH_CUR_PWM:			break;
		case SERVO_PITCH_CUR_PERC:			servo_setPercnet(SERVO_PITCH, (int8_t)*(uint32_t*)value); break;
		case SERVO_PITCH_MIN_ANG:			servo_info.servos[SERVO_PITCH].min_ang = *(uint32_t*)value; break;
		case SERVO_PITCH_CENT_ANG:			servo_info.servos[SERVO_PITCH].center_ang = *(uint32_t*)value; break;
		case SERVO_PITCH_MAX_ANG:			servo_info.servos[SERVO_PITCH].max_ang = *(uint32_t*)value; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}


