


#ifndef INC_MOTOR_SAFETY_H_
#define INC_MOTOR_SAFETY_H_

#include "main.h"
#include "motor_types.h"

extern TIM_HandleTypeDef htim1;


void automatic_motor_switch_off(Motor* m);
void safety_gotostart_blocker(Motor* m);
void motor_safety_stop_motor(void);
void recive_motor_command(void);
void motor_check_parameter(TIM_HandleTypeDef *htim);

#endif