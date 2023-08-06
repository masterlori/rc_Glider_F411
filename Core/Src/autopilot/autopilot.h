/*
 * autopilot.h
 *
 *  Created on: Jul 21, 2023
 *      Author: Usver
 */

#ifndef SRC_AUTOPILOT_AUTOPILOT_H_
#define SRC_AUTOPILOT_AUTOPILOT_H_

enum autopilot_timers
{
	AUTOPILOT_TMR_INIT,
	AUTOPILOT_TMR_UPD,
	AUTOPILOT_TMR_ARM,
	AUTOPILOT_TMR_NUM
};

typedef struct __autopilot_infoTypeDef{
	uint32_t timer[AUTOPILOT_TMR_NUM];
	uint8_t state;
	uint8_t armed_flag;
	uint8_t ailerons_test;
	uint8_t elevator_test;
	uint8_t error_flag;
	uint32_t error_line;
	uint8_t debug_enabled;
}autopilot_infoTypeDef;

extern volatile autopilot_infoTypeDef autopilot_info;
extern void autopilot_Timer(uint32_t res);
extern void autopilot_InitTask(void);
extern void autopilot_Task(void);

//Configurator parameters
enum
{
	AUTOPILOT_STATE,
	AUTOPILOT_ARMED,
	AUTOPILOT_VAR_NUM,
};

extern uint16_t cfg_NodeApVarProp(uint16_t varid, char *name, uint16_t *prop);
extern uint16_t cfg_NodeApVarGet(uint16_t varid, void *value);
extern uint16_t cfg_NodeApVarSet(uint16_t varid, void *value);


#endif /* SRC_AUTOPILOT_AUTOPILOT_H_ */
