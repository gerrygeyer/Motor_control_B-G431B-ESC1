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
#include "main.h"

extern TIM_HandleTypeDef htim2;

void reset_observer(void);
// void openloop(uint16_t speed_rpm, observer *pHandle);

void start_time_measurement(void);
uint32_t stopp_time_measurement(void);

#endif /* INC_OBSERVER_H_ */
