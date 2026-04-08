/*
 * motor_task.h
    *  Created on: Jul 2, 2024
*/

#ifndef INC_MOTOR_TASK_H_
#define INC_MOTOR_TASK_H_   

#include <stdint.h>
#include <stdbool.h>

// Use project-local headers with quotes so they resolve when the compiler or
// language server does not have Core/Inc on the system include path.
#include "foc_math.h"
#include "motor_types.h"
#include "parameter.h"


#define MIDDLE_FREQUENCY_DIVIDER    DIVISION_M         // 20 000 / 20 = 1000 Hz
#define LOW_FREQUENCY_DIVIDER       DIVISION_L       // 20 000 / 100 = 200 Hz

void init_motor_task(void);
void motor_time_management(void);
void out_source_time_management(void);
void init_flags(void);

Control_Loops* Control_GetLoops(void);
FOC_HandleTypeDef* FOC_GetValues(void);

typedef struct{
    bool mid;
    bool low;
    bool oneHz;
}freq_flag_t;

#endif /* INC_MOTOR_TASK_H_ */