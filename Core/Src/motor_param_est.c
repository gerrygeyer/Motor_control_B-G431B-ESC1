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

    
    uint32_t step = estimation_t.counter / estimation_t.time_div;

    switch (step)
    {
        case 0:

        break;
        case 1:

        break;
        case 2:

        break;
        default:
           return;
         break;
    }  
    estimation_t.counter++;

}