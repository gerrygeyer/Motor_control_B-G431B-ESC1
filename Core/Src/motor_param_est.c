/*
 * parameter estimation.c
*/

#include "motor_param_est.h"
#include "foc_math.h"
#include "current_measurement.h"
#include <stddef.h>
#include <stdint.h>
#include "motor_types.h"
#include "parameter.h"

param_estimation_t estimation_t;

/* NOTE:
 * This module only sequences the identification procedure.
 * The actual identification algorithms (signal injection, filtering, fitting)
 * should be implemented in the TODO blocks below.
 */

 static void pidm_enter_idle(Motor* m)
{
    m->pidm_state = PIDM_IDLE;
    m->pidm_result.valid = false;
}

void MotorParamEst_Init(Motor* m)
{
    if (m == NULL) return;
    pidm_enter_idle(m);
    m->pidm_start_request_flag = false;
    m->pidm_abort_request_flag = false;
    estimation_t.counter = 0;
    estimation_t.time_div = RS_ESTIMATION_TIME_PER_STEP;
    estimation_t.Rs = MEASUREMENT;
    for (int i = 0; i < 6; i++) {
        estimation_t.mini_counter[i] = 0;
    }
}

bool MotorParamEst_IsDone(const Motor* m)
{
    if (m == NULL) return false;
    return (m->pidm_state == PIDM_DONE) && (m->pidm_result.valid == true);
}

void MotorParamEst_Service(Motor* m, FOC_HandleTypeDef *foc_values)
{
    if (m == NULL) return;

    /* Run nested SM only in Parameter-ID top-level state */
    if (m->state != ST_PARAMETER_ID) {
        /* If we leave ST_PARAMETER_ID, go back to idle */
        if (m->pidm_state != PIDM_IDLE) {
            pidm_enter_idle(m);
        }
        return;
    }

    /* Abort dominates */
    if (m->pidm_abort_request_flag) {
        m->pidm_abort_request_flag = false;
        m->pidm_state = PIDM_ERROR;
    }

    switch (m->pidm_state)
    {
        case PIDM_IDLE:
            /* Wait for explicit start */
            if (m->pidm_start_request_flag) {
                m->pidm_start_request_flag = false;
                m->pidm_state = PIDM_START;
            }
            break;

        case PIDM_START:
            /* Put motor in a safe, deterministic condition for identification */
            /* Example: disable speed loop, set a known injection mode, reset integrators */
            /* TODO: initialize injections / observers */
            m->pidm_state = PIDM_EST_R;
            break;

        case PIDM_EST_R:
            
            /* Estimate stator resistance Rs */
            foc_values->foc_mode = FOC_CURRENT_CONTROL;

            /* TODO: apply DC current injection / measure V/I / compute Rs */
            /* When finished, store result and move on */
            /* m->pidm_result.Rs_ohm = ...; */
            m->pidm_state = PIDM_EST_L;
            break;

        case PIDM_EST_L:
            /* Estimate inductance Ls */
            /* TODO: apply AC injection / measure response / compute L */
            /* m->pidm_result.Ls_h = ...; */
            m->pidm_state = PIDM_EST_J;
            break;

        case PIDM_EST_J:
            /* Estimate inertia J */
            /* TODO: perform controlled acceleration / use torque estimate vs. alpha */
            /* m->pidm_result.J_kgm2 = ...; */
            m->pidm_result.valid = true;
            m->pidm_state = PIDM_DONE;
            break;

        case PIDM_DONE:
            /* Hold results until user leaves ST_PARAMETER_ID or starts again */
            break;

        case PIDM_ERROR:
        default:
            /* Keep motor safe */
            /* TODO: disable injections / set outputs safe */
            break;
    }
}




static void resistor_measurement_timing(FOC_HandleTypeDef *pHandle_foc){

    static int32_t med_voltage[6] = {0,0,0,0,0,0};
    static int32_t current_injection_q15[6] = {0,0,0,0,0,0};

    const float current_step0 = 0.1f; // A
    const float current_step1 = 0.2f; // A
    const float current_step2 = 0.3f; // A
    const float current_step3 = 0.4f; // A
    const float current_step4 = 0.5f; // A
    const float current_step5 = 0.6f; // A
    
    current_injection_q15[0] = (int16_t)((current_step0 / 32.0f) * (float)Q15); // 50% of max current
    current_injection_q15[1] = (int16_t)((current_step1 / 32.0f) * (float)Q15); // 50% of max current
    current_injection_q15[2] = (int16_t)((current_step2 / 32.0f) * (float)Q15);
    current_injection_q15[3] = (int16_t)((current_step3 / 32.0f) * (float)Q15);
    current_injection_q15[4] = (int16_t)((current_step4 / 32.0f) * (float)Q15);
    current_injection_q15[5] = (int16_t)((current_step5 / 32.0f) * (float)Q15);

    switch (estimation_t.Rs) {
        case MEASUREMENT:
            

    uint16_t step = estimation_t.counter / estimation_t.time_div;
    uint16_t half_time_steps = (estimation_t.counter / (estimation_t.time_div / 2)) - (step * 2);

        switch (step)
        {
            case 0:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = current_injection_q15[0];
                if(half_time_steps != 0){   // take median of first half of the time step to get a stable voltage measurement
                    if(estimation_t.mini_counter[step]  < 100){      
                        med_voltage[step] += pHandle_foc->V_dq_q15.d;
                        estimation_t.mini_counter[step]  ++;
                    }
                }
            break;
            case 1:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = current_injection_q15[1];
                if(half_time_steps != 0){ 
                    if(estimation_t.mini_counter[step]  < 100){      
                    med_voltage[step] += pHandle_foc->V_dq_q15.d;
                    estimation_t.mini_counter[step]  ++;
                    }
            }
            break;
            case 2:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = current_injection_q15[2];
                if(half_time_steps != 0){ 
                if(estimation_t.mini_counter[step]  < 100){      
                    med_voltage[step] += pHandle_foc->V_dq_q15.d;
                    estimation_t.mini_counter[step]  ++;
                }
            }
            break;
            case 3:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = current_injection_q15[3];
                if(half_time_steps != 0){ 
                if(estimation_t.mini_counter[step]  < 100){      
                    med_voltage[step] += pHandle_foc->V_dq_q15.d;
                    estimation_t.mini_counter[step]  ++;
                }
            }
            break;
            case 4:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = current_injection_q15[4];
                if(half_time_steps != 0){ 
                if(estimation_t.mini_counter[step]  < 100){      
                    med_voltage[step] += pHandle_foc->V_dq_q15.d;
                    estimation_t.mini_counter[step]  ++;
                }
            }
            break;
            case 5:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = current_injection_q15[5];
                if(half_time_steps != 0){ 
                if(estimation_t.mini_counter[step]  < 100){      
                    med_voltage[step] += pHandle_foc->V_dq_q15.d;
                    estimation_t.mini_counter[step]  ++;
                }
            }
            break;

            default:
            estimation_t.Rs = CALCULATION;
            // noch mal auf richtige Voltzahl einsetzen.
            for(int i = 0; i < 6; i++){
                med_voltage[i] = med_voltage[i] / (estimation_t.mini_counter[i]); // take average of the voltage measurements
            }
            break;
        }  
           break;
        case CALCULATION:
        lin_reg(current_injection_q15, med_voltage, 6);

        break;
    
    }

}