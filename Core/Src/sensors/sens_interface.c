/*
 * sens_interface.c
 *
 *  Created on: Sep 25, 2023
 *      Author: Usver
 */

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

	return;
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
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT; break;
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_ARMED:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeSensVarGet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case AUTOPILOT_STATE:		*(uint32_t*)value = (uint32_t)autopilot_info.state; break;
		case AUTOPILOT_ARMED:		*(uint32_t*)value = (uint32_t)autopilot_info.armed_flag; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeSensVarSet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case AUTOPILOT_STATE:		autopilot_info.state = (uint8_t)*(uint32_t*)value; break;
		case AUTOPILOT_ARMED:		autopilot_info.armed_flag = (uint8_t)*(uint32_t*)value; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}


