/*
 * task.h
 *
 *  Created on: Dec 24, 2024
 *      Author: gerrygeyer
 */

#ifndef INC_TASK_H_
#define INC_TASK_H_
#include <stm32g4xx_it.h>
#include <stm32g4xx_it.h>
#include "parameter.h"
#include "foc_math.h"


// #########  DEFINE STATES  ##############

#define STOPP_ENGINE			0
#define START_ENGINE			1
#define RUN_ENGINE				2


#define GOTO_START_POSITION 	0
#define RUN_CLOSED_LOOP 		1
#define WAIT 					4
#define OPENLOOP 				2
#define OBSERVER				3
#define STOPP					255

#define OPENLOOP_RAMP			1.0f/(float)FOC_FREQUENCY // -> 1 sec

void init_task(void);
void time_management(void);
float middlespeed_task(int16_t speed_ref);
uint8_t highspeed_task(void);

void engine_mode_observer(void);
void engine_mode_start_stopp(void);
void engine_start_function(void);
void engine_stopp_function(void);
void engine_init_function(void);

void switch_state_observer(void);

void lowspeed_task(void);
void test_function_speed(void);




#endif /* INC_TASK_H_ */
