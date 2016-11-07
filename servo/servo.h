/*
 * servo.h
 *      Author: Sebastian Förster
 *      Website: sebstianfoerster86.wordpress.com
 *      License: GPLv2
 */ 


#ifndef SERVO_H_
#define SERVO_H_

#include "stm32f4xx_hal.h"
#include "tim.h"

#define SERVO_MAXIMUM_ANGLE_US	1700
#define SERVO_MINIMUM_ANGLE_US	1200


#define SERVO_SERVO_0			TIM_CHANNEL_1
#define SERVO_SERVO_1			TIM_CHANNEL_2
#define SERVO_SERVO_2			TIM_CHANNEL_3
#define SERVO_SERVO_3			TIM_CHANNEL_4


typedef struct s_servo
{
	uint16_t max_angle_us;
	uint16_t min_angle_us;
	uint16_t act_angle_us; 	// start pos
	uint16_t final_angle_us; 	// target pos
	uint16_t speed; 			//angle sec
	uint16_t cnt_output_ms;
	uint32_t channel;
} servo;

extern void		servo_init(TIM_HandleTypeDef *htim_in);
extern void 	servo_overflow_IT(void);
extern uint8_t	servo_set_value(uint8_t servo_num, uint16_t new_us_timing, uint16_t new_ms_hold_time, uint16_t angle_sec);
extern uint8_t 	servo_set_speed(uint8_t servo_num, int16_t angle_sec);
extern int 		servo_getpos(uint8_t servo_num);
extern int 		servo_allInPos(void);
extern void 	servo_pause();
extern void 	servo_resume();

#endif /* SERVO_H_ */
