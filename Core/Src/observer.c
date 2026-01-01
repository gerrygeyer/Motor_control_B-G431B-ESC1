/*
 * observer.c
 *
 *  Created on: Jul 2, 2024
 *      Author: Gerry Geyer
 */



#include "parameter.h"
#include <foc_math.h>
#include <settings.h>
#include <string.h>
#include <stdlib.h>
#include <observer.h>



void openloop(uint16_t speed_rpm, observer *pHandle){

	int32_t speed_rps, angle_count;
	uint32_t angle = pHandle->theta_openloop;
	float x1, x2;

	speed_rps = (speed_rpm * PMSM_POLEPAIR)/60;
	x1 = (float)UINT16_MAX_VALUE / (float)FOC_FREQUENCY;
	x2 = (float)speed_rps * x1;
	angle_count = (int32_t)(x2);
	angle = angle + angle_count;

	pHandle->theta_openloop = angle;

}


// function to start, stop the time the motor control need

void start_time_measurement(void){
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
}
// return the operation time in us (uint32_t)
uint32_t stopp_time_measurement(void){

	uint32_t Output;

	TIM2->CR1 &= ~TIM_CR1_CEN;
	uint32_t ticks = TIM2->CNT;
	TIM2->CNT = 0;

	Output = (ticks+85)/170; // scale to us, with +85 we pay attention to correct rounding (170/2 = 85)
	return (Output);
}
