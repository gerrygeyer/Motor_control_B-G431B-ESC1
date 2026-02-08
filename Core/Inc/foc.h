/*
 * foc.h
 *
 *  Created on: Jun 30, 2024
 *      Author: Gerry Geyer
 */

#include "parameter.h"
#include <stdint.h>

//#include <settings.h>



#ifndef SRC_FOC_H_
#define SRC_FOC_H_

#define ON 			1
#define OFF			0


void init_foc(FOC_HandleTypeDef *pHandle_foc);
void clear_foc(FOC_HandleTypeDef *pHandle, Control_Loops *ctrl);
void execute_FOC(FOC_HandleTypeDef *pHandle_foc, Control_Loops *ctrl);

svm_output goto_position(dq_t V);

void generate_openloop(int16_t speed_rpm, FOC_HandleTypeDef *pHandle);


#endif /* SRC_FOC_H_ */
