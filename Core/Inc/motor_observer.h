

#ifndef MOTOR_OBSERVER_H
#define MOTOR_OBSERVER_H


#include "parameter.h"
#include "control.h"
#include "foc.h"
#include "foc_math.h"



void trigger_observer(FOC_HandleTypeDef *pHandle_foc);
void init_observer(void);





#endif /* MOTOR_OBSERVER_H_ */