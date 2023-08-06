

#ifndef SRC_BATTERY_BATTERY_INTERFACE_H_
#define SRC_BATTERY_BATTERY_INTERFACE_H_

#include "main.h"

/* @brief size of battery_meas_arr array */
/* @ref BATTERY_FILT_MEAS_NUM */
#define BATTERY_FILT_MEAS_NUM		8

#define BATTERY_MEAS_COEF			11

/* @brief structure of the battery info  */
typedef struct __battery_infoTypeDef
{
	uint32_t timer;		//count down for the next battery sample
	uint8_t  init_flag; //if 0 indicate initialized process done

	uint8_t  error_flag;
	uint32_t error_line;
	uint8_t  debug_enabled;

	/* @ref battery_volatge_updated_flag */
	uint8_t  battery_volatge_updated_flag; //indicate new voltage reading from the ADC
	uint16_t battery_volatge;			   //holds new voltage reading from the ADC

	// data output from this interface
	/* @ref filtered_voltage */
	uint16_t filtered_voltage;

	/* @ref charge_percent */
	uint8_t  charge_percent;

	uint16_t battery_meas_arr[BATTERY_FILT_MEAS_NUM];
	uint8_t  battery_filter_step;

	/* @brief the priod in mSec to sample the battery voltage */
	/* @ref battery_filtered_measure_period */
	uint32_t battery_filtered_measure_period;

}battery_infoTypeDef;

extern volatile battery_infoTypeDef battery_info;
extern void battery_Timer(uint32_t res);
extern void battery_InitTask(uint32_t battery_filtered_measure_period);
extern void battery_Task(void);

//Configurator parameters
enum
{
	BAT_VOLTAGE,
	BAT_FILTERED_VOLTAGE,
	BAT_CHG_PERC,
	BAT_VAR_NUM,
};

extern uint16_t cfg_NodeBatVarProp(uint16_t varid, char *name, uint16_t *prop);
extern uint16_t cfg_NodeBatVarGet(uint16_t varid, void *value);
extern uint16_t cfg_NodeBatVarSet(uint16_t varid, void *value);




#endif /* SRC_BATTERY_BATTERY_INTERFACE_H_ */
