/*
 * foc.h
 *
 *  Created on: Jun 30, 2024
 *      Author: Gerry Geyer
 */

#include <task.h>
#include "parameter.h"
#include <stdint.h>

//#include <settings.h>



#ifndef SRC_FOC_H_
#define SRC_FOC_H_


#define FROM_ZERO_TO_360		1
#define FROM_MINUS_180_TO_180	0

#define HIGHSPEED 	0
#define LOWSPEED 	1

#define ON 			1
#define OFF			0






void init_foc(FOC_HandleTypeDef *pHandle_foc);
void clear_foc(FOC_HandleTypeDef *pHandle);
alphabeta_f clark_transformation(ab_f Input);
dq_f park_transformation(alphabeta_f I, angle_f elec_theta);
abc_f inverse_clark_transformation(alphabeta_f v);

float PI_controller(float errorstate, uint16_t frequency, float Kp, float Ki,float limit, float old_value);
alphabeta_f inverse_park_transformation(dq_f V, angle_f elec_theta);
void execute_FOC(FOC_HandleTypeDef *pHandle_foc, float Iq);

svm_output goto_position(dq_t V);

void generate_openloop(int16_t speed_rpm, FOC_HandleTypeDef *pHandle);


#endif /* SRC_FOC_H_ */
