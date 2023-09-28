/*
 * sens_interface.c
 *
 *  Created on: Sep 25, 2023
 *      Author: Usver
 */

#include "main.h"
#include "sens_interface.h"
#include "../configurator/cfg_interface.h"
#include "../leds/leds_interface.h"
#include <math.h>

//MPU6050
#define SENS_I2C_ADDR			0x68 << 1
#define SENS_REG_PWR_MGMT_1		0x6B
#define SENS_REG_GYRO_XOUT_H    0x43
#define SENS_REG_GYRO_CONFIG    0x1B
#define SENS_REG_ACCEL_CONFIG   0x1C
#define SENS_REG_CONFIG         0x1A
#define SENS_REG_ACCEL_XOUT_H   0x3B

// Constants for gyro range setup
#define GYRO_RANGE_250DPS    0x00 // ±250°/s
#define GYRO_RANGE_500DPS    0x08 // ±500°/s
#define GYRO_RANGE_1000DPS   0x10 // ±1000°/s
#define GYRO_RANGE_2000DPS   0x18 // ±2000°/s

// Constants for accelerometer range setup
#define ACCEL_RANGE_2G       0x00 // ±2g
#define ACCEL_RANGE_4G       0x08 // ±4g
#define ACCEL_RANGE_8G       0x10 // ±8g
#define ACCEL_RANGE_16G      0x18 // ±16g

// Gyroscope sensitivity constants
#define GYRO_SENS_250DPS     131.0f // Sensitivity at ±250°/s
#define GYRO_SENS_500DPS     65.5f  // Sensitivity at ±500°/s
#define GYRO_SENS_1000DPS    32.8f  // Sensitivity at ±1000°/s
#define GYRO_SENS_2000DPS    16.4f  // Sensitivity at ±2000°/s

// Accelerometer sensitivity constants
#define ACCEL_SENS_2G        16384.0f // Sensitivity at ±2g
#define ACCEL_SENS_4G        8192.0f  // Sensitivity at ±4g
#define ACCEL_SENS_8G        4096.0f  // Sensitivity at ±8g
#define ACCEL_SENS_16G       2048.0f  // Sensitivity at ±16g

//Number of MPU6050 calibration steps
#define SENS_CAL_STEP_NUM		1000

void sens_CalibrateStep();
void sens_WriteByte(uint8_t reg, uint8_t data);
HAL_StatusTypeDef sens_Read(uint8_t reg, uint8_t* data, uint16_t size);
HAL_StatusTypeDef sens_UpdData();
void sens_SetGyroRange(uint8_t range);
void sens_SetAccelRange(uint8_t range);

volatile sens_infoTypeDef sens_info;

void sens_Timer(uint32_t res)
{
#ifdef NO_SENS
	return;
#endif //NO_SENS

	for( uint8_t i = 0; i < SENS_TMR_NUM; i++ )
	{
		if( sens_info.timer[i] > 0 ){
			sens_info.timer[i]--;
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
void sens_InitTask()
{
#ifdef NO_SENS
	return;
#endif //NO_SENS

	//Init state
	sens_info.state = 0;

	return;
}

/**
  * @brief  called from the main loop
  * @param  None
  * @retval None
  */
/**********************************************************************/
void sens_Task(void)
{
#ifdef NO_SENS
	return;
#endif //NO_SENS

	static uint16_t t_cal_step = 0;

	//Reset
	if( sens_info.state == 0 )
	{
		sens_WriteByte(SENS_REG_PWR_MGMT_1, 0x80);
		sens_info.timer[SENS_TMR_PROC] = 5000;
		sens_info.state = 1;
		leds_setBlink(LED_SIM_BLUE, 50, 1000, 0, 0, 1);
	}
	//Init
	else if( sens_info.state == 1 )
	{
		if( sens_info.timer[SENS_TMR_PROC] == 0 )
		{
			//Wake up
			sens_WriteByte(SENS_REG_PWR_MGMT_1, 0x00);
			//Gyroscope config: ±250°/с
			sens_SetGyroRange(GYRO_RANGE_250DPS);
			//Accelerometer config: ±2g
			sens_SetAccelRange(ACCEL_RANGE_2G);
			//Sample Rate: 1kHz
			sens_WriteByte(SENS_REG_CONFIG, 0x00);
			//Prepare for the calibration
			t_cal_step = SENS_CAL_STEP_NUM;
			sens_info.gyro_x_bias = 0.0;
			sens_info.gyro_y_bias = 0.0;
			sens_info.gyro_z_bias = 0.0;
			sens_info.state = 2;
			leds_setBlink(LED_SIM_BLUE, 50, 500, 0, 0, 1);
		}
	}
	//Calibration
	else if( sens_info.state == 2 )
	{
		//Calibration steps
		if( t_cal_step > 0 )
		{
			if( sens_info.timer[SENS_TMR_PROC] == 0 )
			{
				sens_CalibrateStep();
				t_cal_step -= 1;
				sens_info.timer[SENS_TMR_PROC] = 1;
			}
		}
		//Calculating bias
		else
		{
			sens_info.gyro_x_bias /= SENS_CAL_STEP_NUM;
			sens_info.gyro_y_bias /= SENS_CAL_STEP_NUM;
			sens_info.gyro_z_bias /= SENS_CAL_STEP_NUM;
			sens_info.timer[SENS_TMR_PROC] = 1;
			//sens_SetGyroRange(GYRO_RANGE_1000DPS);
			sens_SetAccelRange(ACCEL_RANGE_8G);
			sens_info.state = 3;
			leds_setBlink(LED_SIM_BLUE, 50, 100, 0, 0, 1);
		}
	}
	//Ready
	else if( sens_info.state == 3 )
	{
		if( sens_info.timer[SENS_TMR_PROC] == 0 )
		{
			sens_info.timer[SENS_TMR_PROC] = 1;
			sens_UpdData();
		}
	}

	return;
}

//
void sens_CalibrateStep()
{
    uint8_t buffer[6];
    int16_t gyroX, gyroY, gyroZ;

	HAL_I2C_Mem_Read(&hi2c1, SENS_I2C_ADDR, SENS_REG_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, 1000);
	gyroX = (int16_t)(buffer[0] << 8 | buffer[1]);
	gyroY = (int16_t)(buffer[2] << 8 | buffer[3]);
	gyroZ = (int16_t)(buffer[4] << 8 | buffer[5]);

	sens_info.gyro_x_bias += gyroX;
	sens_info.gyro_y_bias += gyroY;
	sens_info.gyro_z_bias += gyroZ;
}

//
void sens_SetGyroRange(uint8_t range)
{
	sens_info.gyro_range = range;
	sens_WriteByte(SENS_REG_GYRO_CONFIG, range);

    return;
}

//
void sens_SetAccelRange(uint8_t range)
{
	sens_info.accel_range = range;
	sens_WriteByte(SENS_REG_ACCEL_CONFIG, range);

    return;
}

//
HAL_StatusTypeDef sens_UpdData()
{
    uint8_t buf[14];
    HAL_StatusTypeDef status;

    status = sens_Read(SENS_REG_ACCEL_XOUT_H, buf, 14);
    if (status != HAL_OK) {
        return status;
    }

    int16_t raw_accel_x = (int16_t)(buf[0] << 8 | buf[1]);
    int16_t raw_accel_y = (int16_t)(buf[2] << 8 | buf[3]);
    int16_t raw_accel_z = (int16_t)(buf[4] << 8 | buf[5]);
    int16_t raw_temp = (int16_t)(buf[6] << 8 | buf[7]);
    int16_t raw_gyro_x = (int16_t)(buf[8] << 8 | buf[9]);
    int16_t raw_gyro_y = (int16_t)(buf[10] << 8 | buf[11]);
    int16_t raw_gyro_z = (int16_t)(buf[12] << 8 | buf[13]);

    float gyro_sens;
    float accel_sens;

    switch (sens_info.gyro_range) {
        case GYRO_RANGE_250DPS:
            gyro_sens = GYRO_SENS_250DPS;
            break;
        case GYRO_RANGE_500DPS:
            gyro_sens = GYRO_SENS_500DPS;
            break;
        case GYRO_RANGE_1000DPS:
            gyro_sens = GYRO_SENS_1000DPS;
            break;
        case GYRO_RANGE_2000DPS:
            gyro_sens = GYRO_SENS_2000DPS;
            break;
    }
    switch (sens_info.accel_range) {
        case ACCEL_RANGE_2G:
            accel_sens = ACCEL_SENS_2G;
            break;
        case ACCEL_RANGE_4G:
            accel_sens = ACCEL_SENS_4G;
            break;
        case ACCEL_RANGE_8G:
            accel_sens = ACCEL_SENS_8G;
            break;
        case ACCEL_RANGE_16G:
            accel_sens = ACCEL_SENS_16G;
            break;
    }

    sens_info.accel_x = (float)raw_accel_x / accel_sens; // g
    sens_info.accel_y = (float)raw_accel_y / accel_sens; // g
    sens_info.accel_z = (float)raw_accel_z / accel_sens; // g
    sens_info.gyro_x = ((float)raw_gyro_x / gyro_sens) - ((float)sens_info.gyro_x_bias  / gyro_sens); // °/s
    sens_info.gyro_y = ((float)raw_gyro_y / gyro_sens) - ((float)sens_info.gyro_y_bias  / gyro_sens); // °/s
    sens_info.gyro_z = ((float)raw_gyro_z / gyro_sens) - ((float)sens_info.gyro_z_bias  / gyro_sens); // °/s
    sens_info.temperature = ((float)raw_temp / 340.0) + 36.53; // °C

    return HAL_OK;
}

void sens_WriteByte(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&HAL_MEMS_I2C, SENS_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    return;
}

HAL_StatusTypeDef sens_Read(uint8_t reg, uint8_t* data, uint16_t size)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&HAL_MEMS_I2C, SENS_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, size, 1000);
	if (status != HAL_OK) {
		return status;
	}

    return HAL_OK;
}

/* Configurator node functions*/
/*************************************************************************/
uint16_t cfg_NodeSensVarProp(uint16_t varid, char *name, uint16_t *prop)
{
	char *str;

	switch( varid )
	{
		case SENS_STATE:		str = "State"; break;
		case SENS_GYRO_X:		str = "Gyro X"; break;
		case SENS_GYRO_Y:		str = "Gyro Y"; break;
		case SENS_GYRO_Z:		str = "Gyro Z"; break;
		case SENS_ACCEL_X:		str = "Accel X"; break;
		case SENS_ACCEL_Y:		str = "Accel Y"; break;
		case SENS_ACCEL_Z:		str = "Accel Z"; break;
		case SENS_TEMPERATURE:	str = "Temperature"; break;
		case SENS_SAT_NUM:		str = "GNSS sat number"; break;
		case SENS_LATITUDE:		str = "GNSS latitude"; break;
		case SENS_LONGITUDE:	str = "GNSS longitude"; break;
		case SENS_ALTTITUDE:	str = "GNSS altitude"; break;
		case SENS_SPEED:		str = "GNSS speed"; break;
		default: return CFG_ERROR_VARID;
	}
	if( name ) { while( *str ) *name++ = *str++; *name = 0; }

	if( prop ) switch( varid )
	{
		case SENS_STATE:		*prop = CFG_VAR_TYPE_UINT; break;
		case SENS_GYRO_X:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case SENS_GYRO_Y:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case SENS_GYRO_Z:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case SENS_ACCEL_X:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case SENS_ACCEL_Y:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case SENS_ACCEL_Z:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case SENS_TEMPERATURE:	*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case SENS_SAT_NUM:		*prop = CFG_VAR_TYPE_UINT | CFG_VAR_PROP_READONLY; break;
		case SENS_LATITUDE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case SENS_LONGITUDE:	*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case SENS_ALTTITUDE:	*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case SENS_SPEED:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeSensVarGet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case SENS_STATE:		*(uint32_t*)value = sens_info.state; break;
		case SENS_GYRO_X:		*(float*)value = sens_info.gyro_x; break;
		case SENS_GYRO_Y:		*(float*)value = sens_info.gyro_y; break;
		case SENS_GYRO_Z:		*(float*)value = sens_info.gyro_z; break;
		case SENS_ACCEL_X:		*(float*)value = sens_info.accel_x; break;
		case SENS_ACCEL_Y:		*(float*)value = sens_info.accel_y; break;
		case SENS_ACCEL_Z:		*(float*)value = sens_info.accel_z; break;
		case SENS_TEMPERATURE:	*(float*)value = sens_info.temperature; break;
		case SENS_SAT_NUM:		*(int32_t*)value = sens_info.sat_num; break;
		case SENS_LATITUDE:		*(int32_t*)value = sens_info.latitude; break;
		case SENS_LONGITUDE:	*(int32_t*)value = sens_info.longitude; break;
		case SENS_ALTTITUDE:	*(float*)value = sens_info.altitude; break;
		case SENS_SPEED:		*(int32_t*)value = sens_info.speed; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeSensVarSet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case SENS_STATE:		sens_info.state = (uint8_t)*(uint32_t*)value; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}


