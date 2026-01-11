/*
 * Motor saftey functions
 */



#include "motor_safety.h"
#include "motor_types.h"
#include "parameter.h"
#include "settings.h"
#include "motor_task.h"
#include <stdint.h>



/*
 *  a function that stops the motor if the connection is lost
 *  for a certain period of time
 */
void automatic_motor_switch_off(Motor* m){
    if(AUTOMATIC_SWITCH_OFF == OFF) return;
    static uint16_t safety_counter = 0;
    static uint16_t max_count = (uint16_t)(LOSE_CONNECTION_TIME * (float)FOC_FREQUENCY/ (float)LOW_FREQUENCY_DIVIDER);
        if(safety_counter > max_count){
        // Stop the motor
        m->stop_request_flag = true;
        safety_counter = 0;
    }
    safety_counter += 1;
}


/*
 * A function that blocks going into GOTOSTART state for a certain time
 * after finishing the GOTOSTART procedure - the function have to be triggered in the lowspeed task
 */
void safety_gotostart_blocker(Motor* m){
    static uint32_t blocker_counter = 0;
    if(m->gotostart_finish_flag){
        blocker_counter += 1;
        if(blocker_counter >= (FOC_FREQUENCY/LOW_FREQUENCY_DIVIDER * INIT_SAFETY_TIME)){ 
            blocker_counter = 0;
            m->gotostart_finish_flag = false;
        }
    }else{
        blocker_counter = 0;
    }
}