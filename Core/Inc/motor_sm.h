/*
 * motor_sm.h

*/

#ifndef INC_MOTOR_SM_H_
#define INC_MOTOR_SM_H_

#include "motor_types.h"

void MotorSM_Init(Motor* m);
void MTR_Stop(Motor* m);
void MTR_RunClosedLoop(Motor* m);
void MTR_RunOpenLoop(Motor* m);
void MTR_RunSensorless(Motor* m);
void MTR_GotoStart(Motor* m);
void MTR_Fault(Motor* m);

void MotorSM_Init(Motor* m);
void MotorSM_Service(Motor* m);


#endif /* INC_MOTOR_SM_H_ */

    
