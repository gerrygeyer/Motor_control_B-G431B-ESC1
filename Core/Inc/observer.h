/*
 * observer.h
 *
 *  Created on: Jul 2, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_OBSERVER_H_
#define INC_OBSERVER_H_

#include <stdint.h>
#include "parameter.h"
#include <task.h>
#include <foc.h>
#include <main.h>

extern TIM_HandleTypeDef htim2;

// Only for fixed Parameter and Frequency
//#define ONE_DIV_LS_TS 	13.568521		// (1/Ls)*Ts
//#define RS_DIV_LS_TS 	0.8234735		// (Rs/Ls)*Ts

#define ONE_DIV_LS_TS 	0.13568521		// (1/Ls)*Ts
#define RS_DIV_LS_TS 	0.008234735		// (Rs/Ls)*Ts

#define SLOPE_TO_RPM	((float)FOC_FREQUENCY * 60.0f)/((float)UINT16_MAX_VALUE * (float)PMSM_POLEPAIR)

#define INT16_TO_RAD	(float)0.00009587379924285257
#define RAD_TO_INT16	10430 //(float)10430.37835047045
#define RAD_TO_RPM		(float)(60.0f/(PI_MULTIPLY_2 * (float)PMSM_POLEPAIR))

#define ONE_DIV_J_TS	(float)7.250790336146641 	// 1/(15000 * PMSM_SYSTEM_J)
#define TL_MAX_VALUE	0xFFFFFF // 2^24-1 random number


void init_observer(observer *pHandle_ob);
void reset_observer(void);
void openloop(uint16_t speed_rpm, observer *pHandle);

void start_time_measurement(void);
uint32_t stopp_time_measurement(void);

#endif /* INC_OBSERVER_H_ */
