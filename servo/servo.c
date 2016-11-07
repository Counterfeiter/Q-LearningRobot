/*
 * servo.c
 *      Author: Sebastian Förster
 *      Website: sebstianfoerster86.wordpress.com
 *      License: GPLv2
 */ 


#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include "servo.h"

#define SERVO_MS_RESOLUTION		20 //ms overflow interrupt


static volatile servo servos[4];
static int pause = false;

static TIM_HandleTypeDef *htim = NULL;


static void servo_init_structs();
static void servo_set_next_pos(volatile servo *servo);
static uint16_t servo_calc_next_pos(volatile servo *servo);

// use always 16 bit timer to get a better precision
void servo_init(TIM_HandleTypeDef *htim_in)
{
	
	// hal already init the pwm drivers
	// interrupts also active

	servo_init_structs();

	//save timer handle
	htim = htim_in;
	
	
}

uint8_t servo_set_value(uint8_t servo_num, uint16_t new_us_timing, uint16_t new_ms_hold_time, uint16_t angle_sec)
{
	//assert(htim != NULL);

	//assert(servo_num < 4);

	if(new_us_timing < servos[servo_num].min_angle_us) return false;
	if(new_us_timing > servos[servo_num].max_angle_us) return false;

	servos[servo_num].cnt_output_ms = new_ms_hold_time;
	servos[servo_num].final_angle_us = new_us_timing;
	servos[servo_num].speed = angle_sec;

	servo_set_next_pos(&servos[servo_num]);

	pause = false;

	return true;
}

uint8_t servo_set_speed(uint8_t servo_num, int16_t angle_sec)
{
	//assert(htim != NULL);

	//assert(servo_num < 4);

	if(angle_sec == 0)
	{
		//stop servo
		servos[servo_num].final_angle_us = servos[servo_num].act_angle_us;
	}
	else if(angle_sec < 0) {
		//move backward
		servos[servo_num].final_angle_us = servos[servo_num].min_angle_us;
	} else {
		//move forward
		servos[servo_num].final_angle_us = servos[servo_num].max_angle_us;
	}

	servos[servo_num].cnt_output_ms = UINT16_MAX; //pwm forever on!

	servos[servo_num].speed = abs(angle_sec);

	servo_set_next_pos(&servos[servo_num]);

	pause = false;

	return true;
}

int servo_getpos(uint8_t servo_num)
{
	return servos[servo_num].act_angle_us;
}

int servo_allInPos(void)
{
	uint8_t cnt_servos = sizeof(servos) / sizeof(servo);
	for(int i = 0; i < cnt_servos; i++)
	{
		if(servos[i].act_angle_us != servos[i].final_angle_us)
			return false;
	}

	return true;
}

static void servo_set_next_pos(volatile servo *servo)
{
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = servo_calc_next_pos(servo);
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, servo->channel);

	__HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
	HAL_TIM_PWM_Start(htim, servo->channel);
}

static uint16_t servo_calc_next_pos(volatile servo *servo)
{
	int next_angle_step = (int)servo->speed * 5555 / (1000 / SERVO_MS_RESOLUTION); // 1000 / 180 = 5,55 -> * 1000

	int result = servo->act_angle_us;
	if(servo->final_angle_us < servo->act_angle_us) {
		result = (servo->act_angle_us * 1000 - next_angle_step) / 1000;
		result = (result < servo->final_angle_us) ? servo->final_angle_us : result;
	} else if(servo->final_angle_us > servo->act_angle_us) {
		result = (servo->act_angle_us * 1000 + next_angle_step) / 1000;
		result = (result > servo->final_angle_us) ? servo->final_angle_us : result;
	}
	if(result < servo->min_angle_us) result = servo->min_angle_us;
	if(result > servo->max_angle_us) result = servo->max_angle_us;

	servo->act_angle_us = (uint16_t)result;

	return  (uint16_t)result;
}

static void servo_init_structs()
{
	uint8_t cnt_servos = sizeof(servos) / sizeof(servo);
	
	for(uint8_t i = 0; i < cnt_servos; i++)
	{
		servos[i].cnt_output_ms = 0;		// no drive req.
		servos[i].max_angle_us = SERVO_MAXIMUM_ANGLE_US;	//max
		servos[i].min_angle_us = SERVO_MINIMUM_ANGLE_US;	//min
		//start in the middle
		servos[i].act_angle_us = SERVO_MINIMUM_ANGLE_US + ((SERVO_MAXIMUM_ANGLE_US - SERVO_MINIMUM_ANGLE_US) / 2);
	}

	servos[0].channel = SERVO_SERVO_0;
	servos[1].channel = SERVO_SERVO_1;
	servos[2].channel = SERVO_SERVO_2;
	servos[3].channel = SERVO_SERVO_3;
}

//sometimes handy if the CPU is processing something and you want to keep this current positions
void servo_pause()
{
	pause = true;
}

void servo_resume()
{
	pause = false;
}

void servo_overflow_IT(void)
{
	if(pause) return;
	for(uint8_t i = 0; i < 4; i++)
	{
		if(servos[i].cnt_output_ms != 0)
		{
			if(servos[i].cnt_output_ms != UINT16_MAX) servos[i].cnt_output_ms--; // stop the servo interaction if control time is over
			servo_set_next_pos(&servos[i]);
		} else {
			HAL_TIM_PWM_Stop(htim, servos[i].channel);
		}
	}
}
