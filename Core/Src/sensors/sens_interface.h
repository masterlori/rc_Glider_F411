/*
 * sens_interface.h
 *
 *  Created on: Sep 25, 2023
 *      Author: Usver
 */

#ifndef SRC_SENSORS_SENS_INTERFACE_H_
#define SRC_SENSORS_SENS_INTERFACE_H_

enum sens_timers
{
	SENS_TMR_PROC,
	SENS_TMR_NUM
};

typedef struct __sens_infoTypeDef{
	uint32_t timer[SENS_TMR_NUM];
	uint8_t state;
	uint8_t gyro_range;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float gyro_x_bias;
	float gyro_y_bias;
	float gyro_z_bias;
	uint8_t accel_range;
	float accel_x;
	float accel_y;
	float accel_z;
	float temperature;
	uint8_t sat_num;
	int32_t latitude;
	int32_t longitude;
	float altitude;
	int32_t speed;
	uint8_t error_flag;
	uint32_t error_line;
	uint8_t debug_enabled;
}sens_infoTypeDef;

extern volatile sens_infoTypeDef sens_info;
extern void sens_Timer(uint32_t res);
extern void sens_InitTask(void);
extern void sens_Task(void);

//Configurator parameters
enum
{
	SENS_STATE,
	SENS_GYRO_X,
	SENS_GYRO_Y,
	SENS_GYRO_Z,
	SENS_ACCEL_X,
	SENS_ACCEL_Y,
	SENS_ACCEL_Z,
	SENS_TEMPERATURE,
	SENS_SAT_NUM,
	SENS_LATITUDE,
	SENS_LONGITUDE,
	SENS_ALTTITUDE,
	SENS_SPEED,
	SENS_VAR_NUM,
};

extern uint16_t cfg_NodeSensVarProp(uint16_t varid, char *name, uint16_t *prop);
extern uint16_t cfg_NodeSensVarGet(uint16_t varid, void *value);
extern uint16_t cfg_NodeSensVarSet(uint16_t varid, void *value);



#endif /* SRC_SENSORS_SENS_INTERFACE_H_ */
