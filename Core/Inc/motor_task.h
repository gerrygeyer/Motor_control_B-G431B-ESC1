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


#define MIDDLE_FREQUENCY_DIVIDER 20         // 20 000 / 20 = 1000 Hz
#define LOW_FREQUENCY_DIVIDER 200       // 20 000 / 200 = 100 Hz


void motor_time_management(Motor *m);

#endif /* INC_MOTOR_TASK_H_ */