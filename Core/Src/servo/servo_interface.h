/*
 * servo_interface.h
 *
 *  Created on: Jul 16, 2023
 *      Author: Usver
 */

#ifndef SRC_SERVO_SERVO_INTERFACE_H_
#define SRC_SERVO_SERVO_INTERFACE_H_

#define SERVO_1DEG_PWM_STEP	6
#define SERVO_0DEG_PWM_VAL	245

enum servo_timers
{
	SERVO_TX_TMR,
	SERVO_TMR_NUM
};

enum servos_sys
{
	SERVO_ROLL,
	SERVO_PITCH,
	SERVOS_NUM
};

typedef struct __servo_TypeDef
{
	TIM_HandleTypeDef *tim;
	uint32_t timch;
	uint8_t enable;
	uint16_t cur_pwm;
	uint16_t cur_ang;
	int8_t cur_perc;
	uint16_t min_ang;
	uint16_t center_ang;
	uint16_t max_ang;
} servo_TypeDef;

typedef struct __servo_infoTypeDef{
	servo_TypeDef servos[SERVOS_NUM];
	uint32_t timer[SERVO_TMR_NUM];
	uint8_t error_flag;
	uint32_t error_line;
	uint8_t debug_enabled;
}servo_infoTypeDef;

extern volatile servo_infoTypeDef servo_info;
extern void servo_Timer(uint32_t res);
extern void servo_InitTask(void);
extern void servo_Task(void);
extern void servo_setEnable(uint8_t servo, uint8_t ena);
extern void servo_setAng(uint8_t servo, uint16_t ang);
extern void servo_setPercnet(uint8_t servo, int8_t perc);

//Configurator parameters
enum
{
	SERVO_ROLL_ENA,
	SERVO_ROLL_CUR_ANG,
	SERVO_ROLL_CUR_PWM,
	SERVO_ROLL_CUR_PERC,
	SERVO_ROLL_MIN_ANG,
	SERVO_ROLL_CENT_ANG,
	SERVO_ROLL_MAX_ANG,
	SERVO_PITCH_ENA,
	SERVO_PITCH_CUR_ANG,
	SERVO_PITCH_CUR_PWM,
	SERVO_PITCH_CUR_PERC,
	SERVO_PITCH_MIN_ANG,
	SERVO_PITCH_CENT_ANG,
	SERVO_PITCH_MAX_ANG,
	SERVO_VAR_NUM,
};

extern uint16_t cfg_NodeServoVarProp(uint16_t varid, char *name, uint16_t *prop);
extern uint16_t cfg_NodeServoVarGet(uint16_t varid, void *value);
extern uint16_t cfg_NodeServoVarSet(uint16_t varid, void *value);

#endif /* SRC_SERVO_SERVO_INTERFACE_H_ */
