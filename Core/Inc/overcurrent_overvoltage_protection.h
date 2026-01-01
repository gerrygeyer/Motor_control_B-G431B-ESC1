/*
 * overcurrent_overvoltage_protection.h
 *
 *  Created on: Sep 26, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_OVERCURRENT_OVERVOLTAGE_PROTECTION_H_
#define INC_OVERCURRENT_OVERVOLTAGE_PROTECTION_H_

#include <foc.h>



#define DAC1_OUTPUT_LEVEL 120 * OV_MAX_VALUE
#define DAC2_OUTPUT_LEVEL 120 * OV_MAX_VALUE   // (18 / (18 + 169)) * (4096 / 3.3) * V_MAX  ->  119.475f * V_MAX



void overcurrent_occurs(void);


#endif /* INC_OVERCURRENT_OVERVOLTAGE_PROTECTION_H_ */
