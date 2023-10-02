
#include "main.h"
#include "autopilot.h"
#include "../configurator/cfg_interface.h"
#include "../rc/rc_interface.h"
#include "../motor/motor_interface.h"
#include "../servo/servo_interface.h"
#include "../sensors/sens_interface.h"
#include "../crc/crc.h"
#include "usbd_cdc_if.h"
#include <math.h>

volatile autopilot_infoTypeDef autopilot_info;
kalman_TypeDef _roll_filter, _pitch_filter;
PID_TypeDef _roll_pid, _pitch_pid;

uint8_t _ap_arm_st = 0;

void autopilot_gotoState(uint8_t new_state);
void autopilot_stateIdleMain();
void autopilot_stateFullManStart();
void autopilot_stateFullManMain();
void autopilot_stateStabStart();
void autopilot_stateStabMain();
void autopilot_stateFailsafeStart();
void autopilot_stateFailsafeMain();
float autopilot_expRunningAverage(float newVal);
void autopilot_UpdAngles();
void autopilot_KalmanInit(kalman_TypeDef* filter, float Q_angle, float Q_bias, float R_measure);
float autopilot_KalmanUpd(kalman_TypeDef* filter, float newAngle, float newRate, float dt);
int32_t autopilot_CalcPID(PID_TypeDef *pid, float setpoint, float current_angle);
int32_t autopilot_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

enum
{
	AP_STATE_IDLE,
	AP_STATE_FULLMANUAL,
	AP_STATE_STAB,
	AP_STATE_FAILSAFE,
	AP_SATES_NUM,
};

typedef struct __autopilot_sateTypeDef{
	void (*start_func) (void);
	void (*main_func) (void);
	void (*end_func) (void);
	uint32_t upd_period;
}autopilot_stateTypeDef;

autopilot_stateTypeDef autopilot_states[AP_SATES_NUM];
volatile uint8_t _cur_ap_state;

void autopilot_Timer(uint32_t res)
{
#ifdef NO_AUTOPILOT
	return;
#endif //NO_AUTOPILOT

	for( uint8_t i = 0; i < AUTOPILOT_TMR_NUM; i++ )
	{
		if( autopilot_info.timer[i] > 0 ){
			autopilot_info.timer[i]--;
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
void autopilot_InitTask()
{
#ifdef NO_AUTOPILOT
	return;
#endif //NO_AUTOPILOT

	//Init
	_cur_ap_state = 0xFF;
	autopilot_info.timer[AUTOPILOT_TMR_INIT] = 500;
	autopilot_info.armed_flag = 0;
	autopilot_info.astart_elevator_en = 0;
	autopilot_info.astart_motor_en = 0;
	autopilot_info.astart_elevator_val = -80;
	autopilot_info.astart_motor_val = 90;

	autopilot_KalmanInit(&_roll_filter, 0.001, 0.003, 0.03);
	autopilot_KalmanInit(&_pitch_filter, 0.001, 0.003, 0.03);

	//Roll channel default
	_roll_pid.Kp = 1.0f;
	_roll_pid.Ki = 0.1f;
	_roll_pid.Kd = 0.01f;

	//Roll channel default
	_pitch_pid.Kp = 1.0f;
	_pitch_pid.Ki = 0.1f;
	_pitch_pid.Kd = 0.01f;

	//Offsets
	autopilot_info.offset_roll = 0.0f;
	autopilot_info.offset_pitch = 0.0f;

	autopilot_states[AP_STATE_IDLE].start_func = NULL;
	autopilot_states[AP_STATE_IDLE].main_func = autopilot_stateIdleMain;
	autopilot_states[AP_STATE_IDLE].end_func = NULL;
	autopilot_states[AP_STATE_IDLE].upd_period = 0;

	autopilot_states[AP_STATE_FULLMANUAL].start_func = autopilot_stateFullManStart;
	autopilot_states[AP_STATE_FULLMANUAL].main_func = autopilot_stateFullManMain;
	autopilot_states[AP_STATE_FULLMANUAL].end_func = NULL;
	autopilot_states[AP_STATE_FULLMANUAL].upd_period = 10;

	autopilot_states[AP_STATE_STAB].start_func = autopilot_stateStabStart;
	autopilot_states[AP_STATE_STAB].main_func = autopilot_stateStabMain;
	autopilot_states[AP_STATE_STAB].end_func = NULL;
	autopilot_states[AP_STATE_STAB].upd_period = 10;

	autopilot_states[AP_STATE_FAILSAFE].start_func = autopilot_stateFailsafeStart;
	autopilot_states[AP_STATE_FAILSAFE].main_func = autopilot_stateFailsafeMain;
	autopilot_states[AP_STATE_FAILSAFE].end_func = NULL;
	autopilot_states[AP_STATE_FAILSAFE].upd_period = 0;

	return;
}

/**
  * @brief  called from the main loop
  * @param  None
  * @retval None
  */
/**********************************************************************/
void autopilot_Task(void)
{
#ifdef NO_AUTOPILOT
	return;
#endif //NO_AUTOPILOT

	// initialize first state ever
	if( (_cur_ap_state == 0xFF) && (autopilot_info.timer[AUTOPILOT_TMR_INIT] == 0) ){
		autopilot_gotoState(AP_STATE_IDLE);
	}
	else
	{
		if( (_cur_ap_state < AP_SATES_NUM) && (autopilot_states[_cur_ap_state].main_func != NULL) ){
			autopilot_states[_cur_ap_state].main_func();
		}
	}

	if( autopilot_info.timer[AUTOPILOT_TMR_ANG_UPD] == 0 )
	{
		autopilot_info.timer[AUTOPILOT_TMR_ANG_UPD] = 10;
		autopilot_UpdAngles();
	}

	return;
}

/**
  * @brief  sets the new ui state
  * @param  None
  * @retval None
  */
void autopilot_gotoState(uint8_t new_state)
{
#ifdef NO_AUTOPILOT
	return;
#endif //NO_AUTOPILOT

	if( new_state >= AP_SATES_NUM ){
		return;
	}
	if( _cur_ap_state < AP_SATES_NUM )
	{
		if( autopilot_states[_cur_ap_state].end_func != NULL ){
			autopilot_states[_cur_ap_state].end_func();
		}
	}
	if( autopilot_states[new_state].start_func != NULL ){
		autopilot_states[new_state].start_func();
	}

	autopilot_info.timer[AUTOPILOT_TMR_UPD] = autopilot_states[new_state].upd_period;
	_cur_ap_state = new_state;
	autopilot_info.state = new_state;

	return;
}

/*IDLE*/
void autopilot_stateIdleMain()
{
	if( sens_info.state == 4 ){
		autopilot_gotoState(AP_STATE_STAB);
		//autopilot_gotoState(AP_STATE_FULLMANUAL);
	}

	return;
}

/*Full manual control*/
void autopilot_stateFullManStart()
{
	return;
}

void autopilot_stateFullManMain()
{
	float t_trot;
	static uint8_t astart_mot_state = 0;
	static uint8_t astart_elev_state = 0;

	if( autopilot_info.timer[AUTOPILOT_TMR_UPD] == 0 )
	{
		autopilot_info.timer[AUTOPILOT_TMR_UPD] = autopilot_states[_cur_ap_state].upd_period;
		if(rc_info.connected == 1)
		{
			servo_setPercnet(SERVO_ROLL, rc_info.axis_r_y);

			if( autopilot_info.astart_elevator_en == 0 ){
				servo_setPercnet(SERVO_PITCH, rc_info.axis_r_x);
			}
			//Auto start
			else
			{
				servo_setPercnet(SERVO_PITCH, autopilot_info.astart_elevator_val);
				//Auto start switch off condition
				if( abs((int)rc_info.axis_r_x) > 50 ){
					autopilot_info.astart_elevator_en = 0;
				}
			}
			//Motor
			if( ((rc_info.axis_l_x > 0) || (autopilot_info.astart_motor_en == 1)) && (autopilot_info.armed_flag == 1) )
			{
				if( autopilot_info.astart_motor_en == 0 ){
					t_trot = autopilot_expRunningAverage((float)rc_info.axis_l_x);
				}
				//Auto start
				else
				{
					t_trot = autopilot_expRunningAverage((float)autopilot_info.astart_motor_val);
					//Auto start switch off condition
					if( abs((int)rc_info.axis_l_x) > 50 ){
						autopilot_info.astart_motor_en = 0;
					}
				}
			}
			else
			{
				t_trot = autopilot_expRunningAverage(0.0f);
				//motor_setTorque(MOTOR_MAIN, 0);
			}
			motor_setTorque(MOTOR_MAIN, (uint8_t)t_trot);

		}
		//Lost RC reception
		else{
			autopilot_gotoState(AP_STATE_FAILSAFE);
		}
	}

	//ARMING
	if( autopilot_info.armed_flag == 0 )
	{
		if(rc_info.connected == 1)
		{
			//Push trot to min for 3 sec
			if( _ap_arm_st == 0 )
			{
				if( rc_info.axis_l_x < -70 )
				{
					autopilot_info.timer[AUTOPILOT_TMR_ARM] = 3000;
					_ap_arm_st = 1;
				}
			}
			else if( _ap_arm_st == 1 )
			{
				if( rc_info.axis_l_x >= -70 ){
					_ap_arm_st = 0;
				}
				else
				{
					if( autopilot_info.timer[AUTOPILOT_TMR_ARM] == 0 )
					{
						autopilot_info.armed_flag = 1;
						_ap_arm_st = 0;
					}
				}
			}

		}
	}

	//Auto start elevator
	if( autopilot_info.astart_elevator_en == 0 )
	{
		if( astart_elev_state == 0 )
		{
			if( rc_info.button_r == 1 )
			{
				autopilot_info.timer[AUTOPILOT_TMR_ASTART_ELEV] = 2000;
				astart_elev_state = 1;
			}
		}
		else if( astart_elev_state == 1 )
		{
			if( rc_info.button_r != 1 ){
				astart_elev_state = 0;
			}
			else
			{
				if( autopilot_info.timer[AUTOPILOT_TMR_ASTART_ELEV] == 0 )
				{
					autopilot_info.astart_elevator_en = 1;
					astart_elev_state = 0;
				}
			}
		}
	}

	//Auto start motor
	if( (autopilot_info.astart_motor_en == 0) && (autopilot_info.armed_flag == 1) )
	{
		if( astart_mot_state == 0 )
		{
			if( rc_info.button_l == 1 )
			{
				autopilot_info.timer[AUTOPILOT_TMR_ASTART_MOTOR] = 2000;
				astart_mot_state = 1;
			}
		}
		else if( astart_mot_state == 1 )
		{
			if( rc_info.button_l != 1 ){
				astart_mot_state = 0;
			}
			else
			{
				if( autopilot_info.timer[AUTOPILOT_TMR_ASTART_MOTOR] == 0 )
				{
					autopilot_info.astart_motor_en = 1;
					astart_mot_state = 0;
				}
			}
		}
	}

	return;
}

/*Stabilize state*/
void autopilot_stateStabStart()
{
	return;
}

void autopilot_stateStabMain()
{
	float t_trot;

	if( autopilot_info.timer[AUTOPILOT_TMR_UPD] == 0 )
	{
		autopilot_info.timer[AUTOPILOT_TMR_UPD] = autopilot_states[_cur_ap_state].upd_period;
		//Roll
		autopilot_info.tar_roll = autopilot_map((int32_t)rc_info.axis_r_y, -100, 100, -20, 20);
		_roll_pid.output = autopilot_CalcPID(&_roll_pid, autopilot_info.tar_roll, autopilot_info.roll);
		servo_setPercnet(SERVO_ROLL, (int8_t)_roll_pid.output);
		//Pitch
		autopilot_info.tar_pitch = autopilot_map((int32_t)rc_info.axis_r_x, -100, 100, -15, 15);
		_pitch_pid.output = autopilot_CalcPID(&_pitch_pid, autopilot_info.tar_pitch, autopilot_info.pitch);
		servo_setPercnet(SERVO_PITCH, (int8_t)_pitch_pid.output);
		//Motor
		if( autopilot_info.armed_flag == 1 ){
			t_trot = autopilot_expRunningAverage((float)rc_info.axis_l_x);
		}
		else{
			t_trot = autopilot_expRunningAverage(0.0f);
		}
		motor_setTorque(MOTOR_MAIN, (uint8_t)t_trot);

		//ARMING
		if( autopilot_info.armed_flag == 0 )
		{
			if(rc_info.connected == 1)
			{
				//Push trot to min for 3 sec
				if( _ap_arm_st == 0 )
				{
					if( rc_info.axis_l_x < -70 )
					{
						autopilot_info.timer[AUTOPILOT_TMR_ARM] = 3000;
						_ap_arm_st = 1;
					}
				}
				else if( _ap_arm_st == 1 )
				{
					if( rc_info.axis_l_x >= -70 ){
						_ap_arm_st = 0;
					}
					else
					{
						if( autopilot_info.timer[AUTOPILOT_TMR_ARM] == 0 )
						{
							autopilot_info.armed_flag = 1;
							_ap_arm_st = 0;
						}
					}
				}
			}
		}
	}

	return;
}

/*Failsafe state*/
void autopilot_stateFailsafeStart()
{
	servo_setPercnet(SERVO_ROLL, 0);
	servo_setPercnet(SERVO_PITCH, 0);
	motor_setTorque(MOTOR_MAIN, 0);

	return;
}

void autopilot_stateFailsafeMain()
{
	if(rc_info.connected == 1){
		autopilot_gotoState(AP_STATE_FULLMANUAL);
	}
	return;
}

//Kalman filter init
void autopilot_KalmanInit(kalman_TypeDef* filter, float Q_angle, float Q_bias, float R_measure)
{
	filter->Q_angle = Q_angle;
	filter->Q_bias = Q_bias;
	filter->R_measure = R_measure;

	filter->angle = 0;
	filter->bias = 0;

	filter->P[0][0] = 0;
	filter->P[0][1] = 0;
	filter->P[1][0] = 0;
	filter->P[1][1] = 0;

	return;
}

//Kalman filter update
float autopilot_KalmanUpd(kalman_TypeDef* filter, float newAngle, float newRate, float dt)
{
	float S;
	float K[2];
	float y;

	filter->rate = newRate - filter->bias;
	filter->angle += dt * filter->rate;

	filter->P[0][0] += dt * (dt*filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_angle);
	filter->P[0][1] -= dt * filter->P[1][1];
	filter->P[1][0] -= dt * filter->P[1][1];
	filter->P[1][1] += filter->Q_bias * dt;

	S = filter->P[0][0] + filter->R_measure;
	K[0] = filter->P[0][0] / S;
	K[1] = filter->P[1][0] / S;

	y = newAngle - filter->angle;
	filter->angle += K[0] * y;
	filter->bias += K[1] * y;

	filter->P[0][0] -= K[0] * filter->P[0][0];
	filter->P[0][1] -= K[0] * filter->P[0][1];
	filter->P[1][0] -= K[1] * filter->P[0][0];
	filter->P[1][1] -= K[1] * filter->P[0][1];

	return filter->angle;
}

//Update Pitch Roll angles
void autopilot_UpdAngles()
{
	if( sens_info.state == 4 )
	{
		float dt = 0.01;

		float roll = atan2(sens_info.accel_y, sens_info.accel_z) * 180/M_PI;
		float pitch = atan2(-sens_info.accel_x, sqrt(sens_info.accel_y * sens_info.accel_y + sens_info.accel_z * sens_info.accel_z)) * 180/M_PI;

		autopilot_info.roll = autopilot_KalmanUpd(&_roll_filter, roll, sens_info.gyro_x, dt) + autopilot_info.offset_roll;
		autopilot_info.pitch = autopilot_KalmanUpd(&_pitch_filter, pitch, sens_info.gyro_y, dt) + autopilot_info.offset_pitch;
	}
	return;
}

float autopilot_expRunningAverage(float newVal)
{
	static float filVal = 0.0f;

	if( newVal == 0.0f ){
		filVal = 0.0f;
	}
	else{
		filVal += (newVal - filVal) * TROT_FILTER_KOEF;
	}

	return filVal;
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
int32_t autopilot_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to calculate PID control signal
int32_t autopilot_CalcPID(PID_TypeDef *pid, float setpoint, float current_angle) {
    // Calculate error
    float error = setpoint - current_angle;
    // Update integral sum
    pid->integral += error;

    // Limit integral sum to avoid integral windup
    if (pid->integral > 100.0) {
        pid->integral = 100.0;
    } else if (pid->integral < -100.0) {
        pid->integral = -100.0;
    }

    // Calculate PID control signal
    float pid_output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * (error - pid->prev_error));

    // Limit output value between -100 and 100
    if (pid_output > 100.0) {
        pid_output = 100.0;
    } else if (pid_output < -100.0) {
        pid_output = -100.0;
    }

    // Update previous error for the next iteration
    pid->prev_error = error;

    // Convert the result to int32_t and return
    return (int32_t)pid_output;
}

/* Configurator node functions*/
/*************************************************************************/
uint16_t cfg_NodeApVarProp(uint16_t varid, char *name, uint16_t *prop)
{
	char *str;

	switch( varid )
	{
		case AUTOPILOT_STATE:			str = "State"; break;
		case AUTOPILOT_ARMED:			str = "Armed"; break;
		case AUTOPILOT_ROLL:			str = "Roll"; break;
		case AUTOPILOT_OFFSET_ROLL:		str = "Roll offset"; break;
		case AUTOPILOT_PITCH:			str = "Pitch"; break;
		case AUTOPILOT_OFFSET_PITCH:	str = "Pitch offset"; break;
		case AUTOPILOT_TAR_ROLL:		str = "Target roll"; break;
		case AUTOPILOT_TAR_PITCH:		str = "Target pitch"; break;
		default: return CFG_ERROR_VARID;
	}
	if( name ) { while( *str ) *name++ = *str++; *name = 0; }

	if( prop ) switch( varid )
	{
		case AUTOPILOT_STATE:			*prop = CFG_VAR_TYPE_UINT; break;
		case AUTOPILOT_ARMED:			*prop = CFG_VAR_TYPE_BOOL; break;
		case AUTOPILOT_ROLL:			*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_OFFSET_ROLL:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_CONST; break;
		case AUTOPILOT_PITCH:			*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case AUTOPILOT_OFFSET_PITCH:	*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_CONST; break;
		case AUTOPILOT_TAR_ROLL:		*prop = CFG_VAR_TYPE_REAL; break;
		case AUTOPILOT_TAR_PITCH:		*prop = CFG_VAR_TYPE_REAL; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeApVarGet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case AUTOPILOT_STATE:			*(uint32_t*)value = (uint32_t)autopilot_info.state; break;
		case AUTOPILOT_ARMED:			*(uint32_t*)value = (uint32_t)autopilot_info.armed_flag; break;
		case AUTOPILOT_ROLL:			*(float*)value = autopilot_info.roll; break;
		case AUTOPILOT_OFFSET_ROLL:		*(float*)value = autopilot_info.offset_roll; break;
		case AUTOPILOT_PITCH:			*(float*)value = autopilot_info.pitch; break;
		case AUTOPILOT_OFFSET_PITCH:	*(float*)value = autopilot_info.offset_pitch; break;
		case AUTOPILOT_TAR_ROLL:		*(float*)value = autopilot_info.tar_roll; break;
		case AUTOPILOT_TAR_PITCH:		*(float*)value = autopilot_info.tar_pitch; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeApVarSet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case AUTOPILOT_STATE:			autopilot_info.state = (uint8_t)*(uint32_t*)value; break;
		case AUTOPILOT_ARMED:			autopilot_info.armed_flag = (uint8_t)*(uint32_t*)value; break;
		case AUTOPILOT_OFFSET_ROLL:		autopilot_info.offset_roll = *(float*)value; break;
		case AUTOPILOT_OFFSET_PITCH:	autopilot_info.offset_pitch = *(float*)value; break;
		case AUTOPILOT_TAR_ROLL:		autopilot_info.tar_roll = *(float*)value; break;
		case AUTOPILOT_TAR_PITCH:		autopilot_info.tar_pitch = *(float*)value; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeRollChVarProp(uint16_t varid, char *name, uint16_t *prop)
{
	char *str;

	switch( varid )
	{
		case ROLLCH_P_COEF:		str = "P coef"; break;
		case ROLLCH_I_COEF:		str = "I coef"; break;
		case ROLLCH_D_COEF:		str = "D coef"; break;
		case ROLLCH_I_VAL:		str = "I value"; break;
		case ROLLCH_OUTPUT:		str = "Output value"; break;
		default: return CFG_ERROR_VARID;
	}
	if( name ) { while( *str ) *name++ = *str++; *name = 0; }

	if( prop ) switch( varid )
	{
		case ROLLCH_P_COEF:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_CONST; break;
		case ROLLCH_I_COEF:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_CONST; break;
		case ROLLCH_D_COEF:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_CONST; break;
		case ROLLCH_I_VAL:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case ROLLCH_OUTPUT:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeRollChVarGet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case ROLLCH_P_COEF:		*(float*)value = _roll_pid.Kp; break;
		case ROLLCH_I_COEF:		*(float*)value = _roll_pid.Ki; break;
		case ROLLCH_D_COEF:		*(float*)value = _roll_pid.Kd; break;
		case ROLLCH_I_VAL:		*(float*)value = _roll_pid.integral; break;
		case ROLLCH_OUTPUT:		*(int32_t*)value = _roll_pid.output; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeRollChVarSet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case ROLLCH_P_COEF:		_roll_pid.Kp = *(float*)value; break;
		case ROLLCH_I_COEF:		_roll_pid.Ki = *(float*)value; break;
		case ROLLCH_D_COEF:		_roll_pid.Kd = *(float*)value; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodePitchChVarProp(uint16_t varid, char *name, uint16_t *prop)
{
	char *str;

	switch( varid )
	{
		case PITCHCH_P_COEF:		str = "P coef"; break;
		case PITCHCH_I_COEF:		str = "I coef"; break;
		case PITCHCH_D_COEF:		str = "D coef"; break;
		case PITCHCH_I_VAL:			str = "I value"; break;
		case PITCHCH_OUTPUT:		str = "Output value"; break;
		default: return CFG_ERROR_VARID;
	}
	if( name ) { while( *str ) *name++ = *str++; *name = 0; }

	if( prop ) switch( varid )
	{
		case PITCHCH_P_COEF:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_CONST; break;
		case PITCHCH_I_COEF:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_CONST; break;
		case PITCHCH_D_COEF:		*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_CONST; break;
		case PITCHCH_I_VAL:			*prop = CFG_VAR_TYPE_REAL | CFG_VAR_PROP_READONLY; break;
		case PITCHCH_OUTPUT:		*prop = CFG_VAR_TYPE_INT | CFG_VAR_PROP_READONLY; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodePitchChVarGet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case PITCHCH_P_COEF:		*(float*)value = _pitch_pid.Kp; break;
		case PITCHCH_I_COEF:		*(float*)value = _pitch_pid.Ki; break;
		case PITCHCH_D_COEF:		*(float*)value = _pitch_pid.Kd; break;
		case PITCHCH_I_VAL:			*(float*)value = _pitch_pid.integral; break;
		case PITCHCH_OUTPUT:		*(int32_t*)value = _pitch_pid.output; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodePitchChVarSet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case PITCHCH_P_COEF:		_pitch_pid.Kp = *(float*)value; break;
		case PITCHCH_I_COEF:		_pitch_pid.Ki = *(float*)value; break;
		case PITCHCH_D_COEF:		_pitch_pid.Kd = *(float*)value; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

