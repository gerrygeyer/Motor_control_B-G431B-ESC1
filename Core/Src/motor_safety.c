/*
 * Motor saftey functions
 */



#include "motor_safety.h"
#include "motor_types.h"
#include "parameter.h"
#include "settings.h"
#include "motor_task.h"
#include <stdint.h>


uint16_t safety_counter = 0;
/*
 *  a function that stops the motor if the connection is lost
 *  for a certain period of time
 */
void automatic_motor_switch_off(Motor* m){
    if(AUTOMATIC_SWITCH_OFF == OFF) return;
    static uint16_t max_count = (uint16_t)(LOSE_CONNECTION_TIME * (float)FOC_FREQUENCY/ (float)LOW_FREQUENCY_DIVIDER);
        if(safety_counter > max_count){
        // Stop the motor
        m->stop_request_flag = true;
        safety_counter = 0;
    }
    safety_counter += 1;
}

void recive_motor_command(void){
    // Reset the safety counter when a valid command is received
    safety_counter = 0;
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

static void PWM_All_Stop(void)
{
    // Hauptkanäle stoppen
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

    // Komplementärkanäle stoppen
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

static void PWM_All_Start(void)
{
	if(MOTOR_DECOUPLE_CURRENT == OFF) return;
    // Hauptkanäle starten
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // Komplementärkanäle starten
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void motor_safety_stop_motor(void){
    PWM_All_Stop();
}




void motor_check_parameter(TIM_HandleTypeDef *htim){
    int64_t data = ENCODER_PULS_PER_REVOLUTION * 16;
    if(htim->Init.Period != (uint32_t)(data - 1)){
        // Error: wrong timer period for encoder
        Error_Handler();
    }
}