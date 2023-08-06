/*
 * motor_interface.h
 *
 *  Created on: Jul 16, 2023
 *      Author: Usver
 */

#ifndef SRC_MOTOR_MOTOR_INTERFACE_H_
#define SRC_MOTOR_MOTOR_INTERFACE_H_

enum motors_sys
{
	MOTOR_MAIN,
	MOTORS_NUM
};

enum motor_timers
{
	MOTOR_TX_TMR,
	MOTOR_TMR_NUM
};

typedef struct __motor_TypeDef
{
	TIM_HandleTypeDef *tim;
	uint32_t timch;
	uint8_t enable;
	uint16_t cur_pwm;
	uint8_t cur_torq;
	uint16_t idle_pwm;
	uint16_t min_pwm;
	uint16_t max_pwm;
} motor_TypeDef;

typedef struct __motor_infoTypeDef{
	motor_TypeDef motors[MOTORS_NUM];
	uint32_t timer[MOTOR_TMR_NUM];
	uint8_t error_flag;
	uint32_t error_line;
	uint8_t debug_enabled;
}motor_infoTypeDef;

extern volatile motor_infoTypeDef motor_info;
extern void motor_Timer(uint32_t res);
extern void motor_InitTask(void);
extern void motor_Task(void);
extern void motor_setEnable(uint8_t motor, uint8_t ena);
extern void motor_setTorque(uint8_t motor, uint8_t torq);

//Configurator parameters
enum
{
	MOTOR_ENA,
	MOTOR_CUR_TORQ,
	MOTOR_CUR_PWM,
	MOTOR_IDLE_PWM,
	MOTOR_MIN_PWM,
	MOTOR_MAX_PWM,
	MOTOR_VAR_NUM,
};

extern uint16_t cfg_NodeMotorVarProp(uint16_t varid, char *name, uint16_t *prop);
extern uint16_t cfg_NodeMotorVarGet(uint16_t varid, void *value);
extern uint16_t cfg_NodeMotorVarSet(uint16_t varid, void *value);

#endif /* SRC_MOTOR_MOTOR_INTERFACE_H_ */
