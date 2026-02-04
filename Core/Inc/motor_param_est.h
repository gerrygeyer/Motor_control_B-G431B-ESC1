#ifndef MOTOR_PARAM_EST_H
#define MOTOR_PARAM_EST_H

#include <stdbool.h>
#include "motor_types.h"

void MotorParamEst_Init(Motor* m);
bool MotorParamEst_IsDone(const Motor* m);
void MotorParamEst_Service(Motor* m);


#endif /* MOTOR_PARAM_EST_H */