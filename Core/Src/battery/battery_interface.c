/**
 * @defgroup  battery_interface
 * @ref		  battery_interface
 * @author    Neat Tech
 * @brief     Software layer to handle battery voltage. \n
 *            Battery voltage is sampled in the analog interface, When there is new sample, it's being \n
 *            copied to this battery interface @ref battery_volatge and rise \n
 *			  update flag @ref battery_volatge_updated_flag and handled in the task. \n
 *            This interface have array of @ref BATTERY_FILT_MEAS_NUM, \n
 *            Once every @ref battery_filtered_measure_period milli seconds, a new sample enter to the array \n
 *            and new average will be calculated. \n
 *            The filtered values kept in @ref filtered_voltage and @ref charge_percent. \n
 *
 * @{
*/


#include "battery_interface.h"
#include "main.h"
#include "../configurator/cfg_interface.h"



volatile battery_infoTypeDef battery_info;
volatile uint32_t battery_time_counter;

#define BATTERY_LUT_LEN	12
uint16_t _battery_perc_lut[BATTERY_LUT_LEN][2] =
{
		{3000, 1},
		{3300, 10},
		{3600, 50},
		{3700, 60},
		{3750, 70},
		{3790, 75},
		{3830, 80},
		{3870, 85},
		{3920, 90},
		{3970, 95},
		{4100, 97},
		{4200, 100}
};

static void battery_filterInit(uint16_t val);
static int32_t analog_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
uint16_t battery_calcPerc(uint16_t m_volt);

/**
  * @brief  called from SysTick_Handler to update local class time
  * @param  res - time pass in mSec
  * @retval None
  */
void battery_Timer(uint32_t res)
{
#ifdef NO_BATTERY
	return;
#endif //NO_BATTERY

	battery_time_counter += res;

	if( battery_info.timer ){
		battery_info.timer--;
	}

	return;
}

/**
  * @brief  Init the battery interface
  * @param  None
  * @retval 1 if success, 0 if failed
  */
void battery_InitTask(uint32_t battery_filtered_measure_period)
{
#ifdef NO_BATTERY
	return 1;
#endif //NO_BATTERY

	// set minimum of 100mSec, just to be safe
	if ( battery_filtered_measure_period < 100 ){
		battery_filtered_measure_period = 100;
	}

	battery_info.battery_filtered_measure_period = battery_filtered_measure_period;
	battery_info.timer = 500;
	battery_info.init_flag = 1;

	return;
}

/**
  * @brief  called from the main loop
  * @param  None
  * @retval None
  */
void battery_Task()
{
#ifdef NO_BATTERY
	return;
#endif //NO_BATTERY

	// enter only if there is new analog reading of the battery voltage, else there is not reason to make any thing
	if ( battery_info.battery_volatge_updated_flag == 1)
	{
		battery_info.battery_volatge_updated_flag = 0;
		//Initialization - 1st fill the filters array
		//done only once at the first time there is a battery voltage reading
		if( battery_info.init_flag == 1)
		{
			battery_filterInit(battery_info.battery_volatge);
			battery_info.init_flag = 0;
		}
		else if( battery_info.timer == 0 ) // need to sample
		{
			battery_info.timer = battery_info.battery_filtered_measure_period;
			battery_info.battery_filter_step = (battery_info.battery_filter_step + 1) % BATTERY_FILT_MEAS_NUM;
			battery_info.battery_meas_arr[battery_info.battery_filter_step] = battery_info.battery_volatge;

			//Calculating average of last BATTERY_FILT_MEAS_NUM measurement
			uint32_t meas_sum = 0;
			for( uint8_t i = 0; i < BATTERY_FILT_MEAS_NUM; i++ ){
				meas_sum += (uint32_t)battery_info.battery_meas_arr[i];
			}

			battery_info.filtered_voltage = (uint16_t)(meas_sum / BATTERY_FILT_MEAS_NUM);
			battery_info.charge_percent = battery_calcPerc(battery_info.filtered_voltage);
		}
	}

	return;
}

/**
  * @brief  function to init the battery filter array so it will work as expected in the first power up.
  * @param  val - battery voltage to init the filter array value.
  * @retval None
  */
static void battery_filterInit(uint16_t val)
{
#ifdef NO_BATTERY
	return;
#endif //NO_BATTERY

	for( uint8_t i = 0; i < BATTERY_FILT_MEAS_NUM; i++ ){
		battery_info.battery_meas_arr[i] = val;
	}

	battery_info.battery_filter_step = 0;
	battery_info.filtered_voltage = val;
	battery_info.charge_percent = battery_calcPerc(battery_info.filtered_voltage);

	return;
}

/**
  * @brief  function to convert from battery mVolt to battery charge percentages
  * @param  m_volt - battery voltage in mVolt.
  * @retval None
  */
uint16_t battery_calcPerc(uint16_t m_volt)
{
#ifdef NO_BATTERY
	return 100;
#endif //NO_BATTERY

	uint8_t i = 0;
	uint16_t o_perc = 0;

	// minimum voltage
	if( m_volt <= _battery_perc_lut[0][0] ){
		return _battery_perc_lut[0][1];
	}

	// maximum voltage
	if( m_volt >= _battery_perc_lut[BATTERY_LUT_LEN - 1][0] ){
		return _battery_perc_lut[BATTERY_LUT_LEN - 1][1];
	}

	// scan the look up table for needed voltage
	for( i = 0; i < BATTERY_LUT_LEN - 1; i++ )
	{
		if( (m_volt >= _battery_perc_lut[i][0]) && (m_volt <= _battery_perc_lut[i + 1][0]) )
		{
			break;
		}
	}

	o_perc = (uint16_t)analog_map((int32_t)m_volt, (int32_t)_battery_perc_lut[i][0],
									(int32_t)_battery_perc_lut[i + 1][0], (int32_t)_battery_perc_lut[i][1],
									(int32_t)_battery_perc_lut[i + 1][1]);

	return o_perc;
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
static int32_t analog_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Configurator node functions*/
/*************************************************************************/
uint16_t cfg_NodeBatVarProp(uint16_t varid, char *name, uint16_t *prop)
{
	char *str;

	switch( varid )
	{
		case BAT_VOLTAGE:			str = "Voltage"; break;
		case BAT_FILTERED_VOLTAGE:	str = "Filtered voltage"; break;
		case BAT_CHG_PERC:			str = "Charge (%)"; break;
		default: return CFG_ERROR_VARID;
	}
	if( name ) { while( *str ) *name++ = *str++; *name = 0; }

	if( prop ) switch( varid )
	{
		case BAT_VOLTAGE:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_READONLY; break;
		case BAT_FILTERED_VOLTAGE:	*prop = CFG_VAR_TYPE_UINT| CFG_VAR_PROP_READONLY; break;
		case BAT_CHG_PERC:			*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_READONLY; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeBatVarGet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case BAT_VOLTAGE: 			*(uint32_t*)value = (uint32_t)battery_info.battery_volatge; break;
		case BAT_FILTERED_VOLTAGE: 	*(uint32_t*)value = (uint32_t)battery_info.filtered_voltage; break;
		case BAT_CHG_PERC: 			*(uint32_t*)value = (uint32_t)battery_info.charge_percent; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeBatVarSet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case BAT_VOLTAGE:			break;
		case BAT_FILTERED_VOLTAGE:	break;
		case BAT_CHG_PERC:			break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}



