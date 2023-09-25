
#include "main.h"
#include "autopilot.h"
#include "../configurator/cfg_interface.h"
#include "../rc/rc_interface.h"
#include "../motor/motor_interface.h"
#include "../servo/servo_interface.h"
#include "../sensors/sens_interface.h"
#include "../crc/crc.h"
#include <math.h>

volatile autopilot_infoTypeDef autopilot_info;
uint8_t _ap_arm_st = 0;

void autopilot_gotoState(uint8_t new_state);
void autopilot_stateIdleMain();
void autopilot_stateFullManStart();
void autopilot_stateFullManMain();
void autopilot_stateFailsafeStart();
void autopilot_stateFailsafeMain();
float autopilot_expRunningAverage(float newVal);

enum
{
	AP_STATE_IDLE,
	AP_STATE_FULLMANUAL,
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

	autopilot_states[AP_STATE_IDLE].start_func = NULL;
	autopilot_states[AP_STATE_IDLE].main_func = autopilot_stateIdleMain;
	autopilot_states[AP_STATE_IDLE].end_func = NULL;
	autopilot_states[AP_STATE_IDLE].upd_period = 0;

	autopilot_states[AP_STATE_FULLMANUAL].start_func = autopilot_stateFullManStart;
	autopilot_states[AP_STATE_FULLMANUAL].main_func = autopilot_stateFullManMain;
	autopilot_states[AP_STATE_FULLMANUAL].end_func = NULL;
	autopilot_states[AP_STATE_FULLMANUAL].upd_period = 10;

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
	autopilot_gotoState(AP_STATE_FULLMANUAL);

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
/*
#include <math.h>

// Данные для фильтра Калмана
typedef struct {
    float Q_angle;  // Процессный шум
    float Q_bias;   // Шум смещения
    float R_measure; // Оценка шума измерений

    float angle;  // Начальный угол
    float bias;   // Начальное смещение

    float rate;   // Непрофильтрованное значение скорости

    float P[2][2]; // Матрица ошибки
} Kalman;

// Инициализация фильтра Калмана
void Kalman_Init(Kalman* filter, float Q_angle, float Q_bias, float R_measure) {
    filter->Q_angle = Q_angle;
    filter->Q_bias = Q_bias;
    filter->R_measure = R_measure;

    filter->angle = 0;
    filter->bias = 0;

    filter->P[0][0] = 0;
    filter->P[0][1] = 0;
    filter->P[1][0] = 0;
    filter->P[1][1] = 0;
}

// Обновление фильтра Калмана
float Kalman_Update(Kalman* filter, float newAngle, float newRate, float dt) {
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

// Пример использования:
int main() {
    Kalman rollFilter, pitchFilter;
    float ax, ay, az, gx, gy, gz;  // значения акселерометра и гироскопа

    // Инициализация фильтров
    Kalman_Init(&rollFilter, 0.001, 0.003, 0.03);
    Kalman_Init(&pitchFilter, 0.001, 0.003, 0.03);

    // Допустим, вы читаете данные из датчиков здесь...
    // ax = ...;
    // ay = ...;
    // az = ...;
    // gx = ...;
    // gy = ...;
    // gz = ...;

    float dt = 0.01; // Временной интервал (зависит от вашей частоты считывания данных)

    float roll = atan2(ay, az) * 180/M_PI;
    float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180/M_PI;

    roll = Kalman_Update(&rollFilter, roll, gx, dt);
    pitch = Kalman_Update(&pitchFilter, pitch, gy, dt);

    // Здесь roll и pitch содержат отфильтрованные углы крена и тангажа соответственно
    return 0;
}

*/
/* Configurator node functions*/
/*************************************************************************/
uint16_t cfg_NodeApVarProp(uint16_t varid, char *name, uint16_t *prop)
{
	char *str;

	switch( varid )
	{
		case AUTOPILOT_STATE:	str = "State"; break;
		case AUTOPILOT_ARMED:	str = "Armed"; break;
		default: return CFG_ERROR_VARID;
	}
	if( name ) { while( *str ) *name++ = *str++; *name = 0; }

	if( prop ) switch( varid )
	{
		case AUTOPILOT_STATE:		*prop = CFG_VAR_TYPE_UINT; break;
		case AUTOPILOT_ARMED:		*prop = CFG_VAR_TYPE_BOOL; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeApVarGet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case AUTOPILOT_STATE:		*(uint32_t*)value = (uint32_t)autopilot_info.state; break;
		case AUTOPILOT_ARMED:		*(uint32_t*)value = (uint32_t)autopilot_info.armed_flag; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

uint16_t cfg_NodeApVarSet(uint16_t varid, void *value)
{
	if( value ) switch( varid )
	{
		case AUTOPILOT_STATE:		autopilot_info.state = (uint8_t)*(uint32_t*)value; break;
		case AUTOPILOT_ARMED:		autopilot_info.armed_flag = (uint8_t)*(uint32_t*)value; break;
		default: return CFG_ERROR_VARID;
	}
	return CFG_ERROR_NONE;
}

