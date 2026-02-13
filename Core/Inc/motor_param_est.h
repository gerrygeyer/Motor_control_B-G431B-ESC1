#ifndef MOTOR_PARAM_EST_H
#define MOTOR_PARAM_EST_H

#include <stdbool.h>
#include "motor_types.h"
#include "parameter.h"

void MotorParamEst_Init(Motor* m);
bool MotorParamEst_IsDone(const Motor* m);
void MotorParamEst_Service(Motor* m, FOC_HandleTypeDef *foc_values);



#define RS_ESTIMATION_TIME_PER_STEP (FOC_FREQUENCY/2) // 0.5 seconds per step

#endif /* MOTOR_PARAM_EST_H */