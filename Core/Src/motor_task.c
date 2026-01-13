/* motor_task.c
 *
 *  Created on: Dec 24, 2024
 */
#include "motor_types.h"
#include "motor_safety.h"
#include "task.h"
#include "motor_task.h"
#include "current_measurement.h"
#include "main.h"
#include "observer.h"
#include "foc.h"
#include "foc_math.h"
#include "parameter.h"
#include "control.h"
#include "svm.h"
#include "communication.h"
#include "encoder.h"
#include <stdint.h>

static void highspeed_motor_task(Motor *m);
static void middlespeed_motor_task(Motor *m);
static void lowspeed_motor_task(Motor *m);

FOC_HandleTypeDef foc_values;

void init_motor_task(void)
{
    init_foc(&foc_values);
    init_control_functions();
    init_SVM();
    init_current_measurement();
    init_encoder();
    init_communication();
    init_math();
}


void motor_time_management(void)
{
    static uint16_t middlespeed_task_counter = 0;
    static uint16_t lowspeed_task_counter = 0;

    highspeed_motor_task(&g_motor);


    // Middlespeed task
    if(middlespeed_task_counter >= MIDDLE_FREQUENCY_DIVIDER){
        middlespeed_task_counter = 0;
        middlespeed_motor_task(&g_motor);
        
    }else{
        middlespeed_task_counter += 1;
    }

    // Lowspeed task
    if(lowspeed_task_counter >= LOW_FREQUENCY_DIVIDER){
        lowspeed_task_counter = 0;
        lowspeed_motor_task(&g_motor);
    }else{
        lowspeed_task_counter += 1;
    }


}


static void highspeed_motor_task(Motor *m)
{
    switch (m->state)
    {
        svm_output pwm_output;

        case ST_STOP:
            // Handle stop state
            reset_pi_integrator();

            // set outputs to zero
            TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
            break;
        case ST_CLOSEDLOOP:
         // Handle closed loop state
            foc_values.foc_mode 	= FOC_CLOSELOOP;
            execute_FOC(&foc_values,0.0f); // Iq muss noch weg 
            pwm_output = get_PWM_OUTPUT_Q15(foc_values.V_q15);
            TIM1->CCR1 = pwm_output.Sa;
			TIM1->CCR2 = pwm_output.Sb;
			TIM1->CCR3 = pwm_output.Sc;
            break;
        case ST_OPENLOOP:
            // Handle open loop state

            foc_values.foc_mode 		= FOC_OPENLOOP;
            if(m->speed_ref > 0){
            	generate_openloop(500, &foc_values);
            }else{
            	generate_openloop(-500, &foc_values);
            }
            execute_FOC(&foc_values, 0.0f); // Iq muss noch weg
            pwm_output = get_PWM_OUTPUT_Q15(foc_values.V_q15);
            TIM1->CCR1 = pwm_output.Sa;
            TIM1->CCR2 = pwm_output.Sb; 
            TIM1->CCR3 = pwm_output.Sc;
            break;
        case ST_GOTOSTART:
            // Handle go to start state
            if(m->gotostart_finish_flag){
                // already at start
                break;
            }
            dq_t V = {0,0};
            V.d = (GOTOSTART_Q_VOLTAGE / (float)MAX_VOLTAGE); // *2.0f
            V.q = 0.0f;
            static uint16_t gotostart_counter = 0;
            if(gotostart_counter < 5000){
                if(gotostart_counter < 4000){
                    set_encoder_to_zero(&foc_values);
                }
                    pwm_output = goto_position(V);
                    set_encoder_to_zero(&foc_values);
                    TIM1->CCR1 = pwm_output.Sa;
                    TIM1->CCR2 = pwm_output.Sb;
                    TIM1->CCR3 = pwm_output.Sc;

                    gotostart_counter++;
            }else{
                gotostart_counter = 0;
                m->gotostart_finish_flag = true;
            }
            break;
        case ST_PARAMETER_ID:
            // Handle fault state
            TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
            break;
        case ST_FAULT:
            // Handle fault state
            TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;

            motor_safety_stop_motor();
            break;
        default:
            reset_pi_integrator();
            // set outputs to zero
            TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
            motor_safety_stop_motor();
            break;
    }
}

static void middlespeed_motor_task(Motor *m)
{
    speed_calculation(&foc_values);
    pi_speed_q15(foc_values.speed,m->speed_ref,&foc_values);
    automatic_switch_off();
	data_log_speed_current_500Hz(&foc_values, (uint8_t)m->state); // Send data back via UART (Optional)
}

static void lowspeed_motor_task(Motor *m)
{
    /**
     * @brief Reads the voltage value and updates the FOC (Field-Oriented Control) values structure.
     * 
     * This function acquires the current voltage measurement from the motor control system
     * and stores the result in the provided FOC values structure for use in field-oriented
     * control calculations.
     * 
     * @param[in,out] foc_values Pointer to the FOC values structure where the read voltage
     *                            value will be stored.
     * 
     * @note This function is declared in current_measurement.h
     * 
     * @see current_measurement.h
     */
    safety_gotostart_blocker(m);       // 
    read_voltage_value(&foc_values);
    execute_voltage_measurement();
    // Implement lowspeed task logic here
    // e.g., position control, etc.

}



void fault_motor_task(Motor *m)
{
    m->state = ST_FAULT;
    m->speed_ref = 0;
}