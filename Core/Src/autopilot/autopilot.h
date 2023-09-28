/*
 * autopilot.h
 *
 *  Created on: Jul 21, 2023
 *      Author: Usver
 */

#ifndef SRC_AUTOPILOT_AUTOPILOT_H_
#define SRC_AUTOPILOT_AUTOPILOT_H_

#define TROT_FILTER_KOEF	0.1

enum autopilot_timers
{
	AUTOPILOT_TMR_INIT,
	AUTOPILOT_TMR_UPD,
	AUTOPILOT_TMR_ANG_UPD,
	AUTOPILOT_TMR_ARM,
	AUTOPILOT_TMR_ASTART_MOTOR,
	AUTOPILOT_TMR_ASTART_ELEV,
	AUTOPILOT_TMR_NUM
};

typedef struct __autopilot_infoTypeDef{
	uint32_t timer[AUTOPILOT_TMR_NUM];
	uint8_t state;
	uint8_t armed_flag;
	uint8_t ailerons_test;
	uint8_t elevator_test;
	uint8_t astart_elevator_en;
	uint8_t astart_motor_en;
	int8_t astart_elevator_val;
	uint8_t astart_motor_val;
	float roll;
	float pitch;
	uint8_t error_flag;
	uint32_t error_line;
	uint8_t debug_enabled;
}autopilot_infoTypeDef;

typedef struct {
    float Q_angle;  // Процессный шум
    float Q_bias;   // Шум смещения
    float R_measure; // Оценка шума измерений

    float angle;  // Начальный угол
    float bias;   // Начальное смещение

    float rate;   // Непрофильтрованное значение скорости

    float P[2][2]; // Матрица ошибки
} kalman_TypeDef;

extern volatile autopilot_infoTypeDef autopilot_info;
extern void autopilot_Timer(uint32_t res);
extern void autopilot_InitTask(void);
extern void autopilot_Task(void);

//Configurator parameters
enum
{
	AUTOPILOT_STATE,
	AUTOPILOT_ARMED,
	AUTOPILOT_ROLL,
	AUTOPILOT_PITCH,
	AUTOPILOT_VAR_NUM,
};

extern uint16_t cfg_NodeApVarProp(uint16_t varid, char *name, uint16_t *prop);
extern uint16_t cfg_NodeApVarGet(uint16_t varid, void *value);
extern uint16_t cfg_NodeApVarSet(uint16_t varid, void *value);


#endif /* SRC_AUTOPILOT_AUTOPILOT_H_ */
