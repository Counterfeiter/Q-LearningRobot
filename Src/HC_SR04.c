/*
 * HC_SR04.c
 *
 *  Created on: 18.10.2016
 *      Author: Sebastian Förster
 *      Website: sebstianfoerster86.wordpress.com
 *      License: GPLv2
 */

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "HC_SR04.h"

static volatile int hcsr04_timestamp = 0;
static volatile int overflow_flag = 0;

//timer PWM (DC 0,5) falling edge at overflow (trigger pin) -> 20 ms interval = 20000 digits
//and falling edge gpio interrupt (echo pin)
//ready to run
//if not, set it up in this function
void hcsr04_startMeasure(void)
{
	//start the timer, to measure the echo flight time right after the falling trigger edge
	__HAL_TIM_ENABLE_IT(&htim12, TIM_IT_UPDATE);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
}

//returns actual distance or if a error occurs 0 mm
uint32_t hcsr04_getLastDistance_mm(void)
{
	if(hcsr04_timestamp == 0) return 0; // timer overflow signal -> problem with measurement
	int ret = ((hcsr04_timestamp * 343) / 1000 - 154) / 2; //mm per 10 us - 154 mm offset -> double distance => / 2
	return ret < 0 ? 0 : ret > 2000 ? 0 : ret;
}

//callback, use it in timer overflow interrupt
void hcsr04_cb_timeroverflow(void)
{
	//check for sensor error (200 ms timeout)
	overflow_flag++;
}

//callback, use it in pin interrupt (falling edge)
void hcsr04_cb_pin_fallingedge(void)
{
	if(overflow_flag > 1 || overflow_flag < 0)
	{
		//discard one more measure after 200 ms timeout
		overflow_flag = -1;
		hcsr04_timestamp = 0;
	} else {
		//save the value of the timer, to calculate the distance
		hcsr04_timestamp = (overflow_flag != 1) ? 0 : htim12.Instance->CNT;

		//reset overflow flag
		overflow_flag = 0;
	}
}
