/*
 * encoder.h
 *
 *  Created on: Aug 6, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <main.h>
#include <foc_math.h>


#define INIT_EL_ANGLE 0  // Degree
#define COUNTERCLOCKWISE -1
#define CLOCKWISE 1
#define COUNT_TO_EL_ANGLE (uint32_t)(((65535 * PMSM_POLEPAIR) + (ENCODER_PULS_PER_REVOLUTION/2))/ENCODER_PULS_PER_REVOLUTION) // 98



void init_encoder(void);
void set_encoder_to_zero(FOC_HandleTypeDef *pHandle_foc);
void calc_rotor_position(FOC_HandleTypeDef *pHandle_foc);
int16_t speed_calculation(FOC_HandleTypeDef *pHandle_foc);
#endif /* INC_ENCODER_H_ */
