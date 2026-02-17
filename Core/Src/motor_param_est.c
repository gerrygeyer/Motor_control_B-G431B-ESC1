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
#include "motor_events.h"
#include "motor_hfi.h"
#include <math.h>

static Status resistor_measurement_timing(FOC_HandleTypeDef *pHandle_foc);
static Status induction_measurement_timing(FOC_HandleTypeDef *pHandle_foc);

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


    const float current_step0 = 0.2f; // A
    const float current_step1 = 0.3f; // A
    const float current_step2 = 0.4f; // A
    const float current_step3 = 0.5f; // A
    const float current_step4 = 0.6f; // A
    const float current_step5 = 0.7f; // A


    const float max_hfi_injection_voltage = 0.05f; // V, for Ls estimation


    estimation_t.counter_Rs = 0;
    estimation_t.time_div_Rs = RS_ESTIMATION_TIME_PER_STEP;
    estimation_t.Rs = MEASUREMENT;
    for (int i = 0; i < 6; i++) {
        estimation_t.mini_counter_Rs[i] = 0;
        estimation_t.med_voltage_Rs[i] = 0;
        estimation_t.current_injection_q15[i] = 0;
    }
    


    estimation_t.current_injection_q15[0] = (int16_t)((current_step0 / 32.0f) * (float)Q15); 
    estimation_t.current_injection_q15[1] = (int16_t)((current_step1 / 32.0f) * (float)Q15); 
    estimation_t.current_injection_q15[2] = (int16_t)((current_step2 / 32.0f) * (float)Q15);
    estimation_t.current_injection_q15[3] = (int16_t)((current_step3 / 32.0f) * (float)Q15);
    estimation_t.current_injection_q15[4] = (int16_t)((current_step4 / 32.0f) * (float)Q15);
    estimation_t.current_injection_q15[5] = (int16_t)((current_step5 / 32.0f) * (float)Q15);

    for(int i = 0; i < 3; i++) {
        estimation_t.mini_counter_Ls[i] = 0;
        estimation_t.med_current_Ls[i] = 0;
    }
    estimation_t.hfi_angle_increment = 0;
    estimation_t.counter_Ls = 0;
    estimation_t.time_div_Ls = LS_ESTIMATION_TIME_PER_STEP;
    estimation_t.Ls = MEASUREMENT;
    estimation_t.hfi_voltage_q15 = (int16_t)(max_hfi_injection_voltage * (float)Q15); // 0.1 * Vmax

    

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
            // /* Wait for explicit start */
            // if (m->pidm_start_request_flag) {
            //     m->pidm_start_request_flag = false;
            //     m->pidm_state = PIDM_START;
            // }
            m->pidm_state = PIDM_START;
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
            if (resistor_measurement_timing(foc_values) == PROCESS_SUCCESS) {
                /* TODO: apply DC current injection / measure V/I / compute Rs */
                /* When finished, store result and move on */
                /* m->pidm_result.Rs_ohm = ...; */
                foc_values->V_abc_q15 = (abc_16t){0,0,0}; // set voltage to zero after estimation is done
                m->pidm_state = PIDM_EST_L;
            }

            break;

        case PIDM_EST_L:
            /* Estimate inductance Ls */
            foc_values->foc_mode = FOC_HFI;
            if(induction_measurement_timing(foc_values) == PROCESS_SUCCESS) {
            /* TODO: apply AC injection / measure response / compute L */
            /* m->pidm_result.Ls_h = ...; */
            foc_values->V_abc_q15 = (abc_16t){0,0,0};
            m->pidm_state = PIDM_EST_J;
            }
            break;

        case PIDM_EST_J:
            /* Estimate inertia J */
            /* TODO: perform controlled acceleration / use torque estimate vs. alpha */
            /* m->pidm_result.J_kgm2 = ...; */
            m->pidm_result.valid = true;
            m->pidm_state = PIDM_DONE;
            break;

        case PIDM_DONE:
            btn_stop_edge = true;
            /* Hold results until user leaves ST_PARAMETER_ID or starts again */
            break;

        case PIDM_ERROR:
            btn_stop_edge = true;
        default:
            /* Keep motor safe */
            /* TODO: disable injections / set outputs safe */
            break;
    }
}


float debug_voltage1, debug_voltage2, debug_voltage3, debug_voltage4, debug_voltage5, debug_voltage6;

static Status resistor_measurement_timing(FOC_HandleTypeDef *pHandle_foc){


    switch (estimation_t.Rs) {
        case MEASUREMENT:
            

        uint16_t step = estimation_t.counter_Rs / estimation_t.time_div_Rs;
        uint16_t half_time_steps = (estimation_t.counter_Rs / (estimation_t.time_div_Rs / 2)) - (step * 2);

        switch (step)
        {
            case 0:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = estimation_t.current_injection_q15[0];
                if(half_time_steps != 0){   // take median of first half of the time step to get a stable voltage measurement
                    if(estimation_t.mini_counter_Rs[step]  < 100){      
                        estimation_t.med_voltage_Rs[step] += pHandle_foc->V_dq_q15.d;
                        estimation_t.mini_counter_Rs[step]  ++;
                    }
                }
            break;
            case 1:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = estimation_t.current_injection_q15[1];
                if(half_time_steps != 0){ 
                    if(estimation_t.mini_counter_Rs[step]  < 100){      
                    estimation_t.med_voltage_Rs[step] += pHandle_foc->V_dq_q15.d;
                    estimation_t.mini_counter_Rs[step]  ++;
                    }
            }
            break;
            case 2:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = estimation_t.current_injection_q15[2];
                if(half_time_steps != 0){ 
                if(estimation_t.mini_counter_Rs[step]  < 100){      
                    estimation_t.med_voltage_Rs[step] += pHandle_foc->V_dq_q15.d;
                    estimation_t.mini_counter_Rs[step]  ++;
                }
            }
            break;
            case 3:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = estimation_t.current_injection_q15[3];
                if(half_time_steps != 0){ 
                if(estimation_t.mini_counter_Rs[step]  < 100){      
                    estimation_t.med_voltage_Rs[step] += pHandle_foc->V_dq_q15.d;
                    estimation_t.mini_counter_Rs[step]  ++;
                }
            }
            break;
            case 4:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = estimation_t.current_injection_q15[4];
                if(half_time_steps != 0){ 
                if(estimation_t.mini_counter_Rs[step]  < 100){      
                    estimation_t.med_voltage_Rs[step] += pHandle_foc->V_dq_q15.d;
                    estimation_t.mini_counter_Rs[step]  ++;
                }
            }
            break;
            case 5:
                pHandle_foc->I_ref_q15.q = 0;
                pHandle_foc->I_ref_q15.d = estimation_t.current_injection_q15[5];
                if(half_time_steps != 0){ 
                if(estimation_t.mini_counter_Rs[step]  < 100){      
                    estimation_t.med_voltage_Rs[step] += pHandle_foc->V_dq_q15.d;
                    estimation_t.mini_counter_Rs[step]  ++;
                }
            }
            break;

            default:
            estimation_t.Rs = CALCULATION;
            for(int i = 0; i < 6; i++){
                estimation_t.med_voltage_Rs[i] = estimation_t.med_voltage_Rs[i] / (estimation_t.mini_counter_Rs[i]); // take average of the voltage measurements
                // Output Voltage are depending on the source voltage, to get the real voltage we have to multiply the measured voltage with the source voltage and divide by Q15 (because the voltage is in Q15)
                estimation_t.med_voltage_Rs[i] = (estimation_t.med_voltage_Rs[i] * pHandle_foc->source_voltage) >> 15; // now the voltage are scaled into (V / 32) * Q15.

                debug_voltage1 = (float)estimation_t.med_voltage_Rs[0] * 32.0f / (float)Q15; // for debugging: convert back to real voltage in V
                debug_voltage2 = (float)estimation_t.med_voltage_Rs[1] * 32.0f / (float)Q15;
                debug_voltage3 = (float)estimation_t.med_voltage_Rs[2] * 32.0f / (float)Q15;
                debug_voltage4 = (float)estimation_t.med_voltage_Rs[3] * 32.0f / (float)Q15;
                debug_voltage5 = (float)estimation_t.med_voltage_Rs[4] * 32.0f / (float)Q15;
                debug_voltage6 = (float)estimation_t.med_voltage_Rs[5] * 32.0f / (float)Q15;
            }
            break;
        }  
        estimation_t.counter_Rs++;
        break;
        case CALCULATION:
        pHandle_foc->I_ref_q15.q = 0;
        pHandle_foc->I_ref_q15.d = 0;
        estimation_t.est_Rs = lin_reg(estimation_t.current_injection_q15, estimation_t.med_voltage_Rs, 6);

        estimation_t.est_Rs = estimation_t.est_Rs/2.0f; // Rs = Rab / 2 because we are measuring line to line voltage but we need phase voltage for the calculation of Rs
        
        return PROCESS_SUCCESS; // estimation done
        break;


        default:
        break;
    
    }
    return PROCESS_UNSUCCESS;

}


float lin_reg(const int32_t *x, const int32_t *y, uint32_t n)
{
    if (x == NULL || y == NULL || n == 0U)
        return 0.0f;

    int64_t sum_xy = 0;  // Q20
    int64_t sum_xx = 0;  // Q20

    for (uint32_t i = 0; i < n; i++)
    {
        int32_t xi = x[i];   // Q10
        int32_t yi = y[i];   // Q10

        sum_xy += (int64_t)xi * yi;  // Q20
        sum_xx += (int64_t)xi * xi;  // Q20
    }

    if (sum_xx == 0)
        return 0.0f;

    /*
     * Since both numerator and denominator are Q20,
     * their ratio is already the real (dimensionless) value.
     * No additional scaling required.
     */
    return (float)sum_xy / (float)sum_xx;
}

float debug_induction1, debug_induction2, debug_induction3;
static Status induction_measurement_timing(FOC_HandleTypeDef *pHandle_foc){

    uint16_t hfi_injection_freq[3];

    const uint16_t hfi_injection_freq1 = 800; // Hz
    const uint16_t hfi_injection_freq2 = 1000; // Hz
    const uint16_t hfi_injection_freq3 = 1200; // Hz

    hfi_injection_freq[0] = hfi_injection_freq1;
    hfi_injection_freq[1] = hfi_injection_freq2;
    hfi_injection_freq[2] = hfi_injection_freq3;

    static iir_filter_f x_d;
    static iir_filter_f x_q;
    static uint8_t initialized = 0;

    if(!initialized) {
        float fc = 1.0f; // Hz, example cutoff frequency for the IIR filter
        float Ts = 1.0f / (float)FOC_FREQUENCY; // Sampling time based on FOC frequency
        float a = (PI_MULTIPLY_2* fc * Ts) / (PI_MULTIPLY_2 * fc * Ts + 1.0f); // Calculate filter coefficient for a first-order low-pass IIR filter
        
        x_d.a = a;
        x_d.one_min_a = 1.0f - x_d.a;
        x_d.y_last = 0;

        x_q.a = a; 
        x_q.one_min_a = 1.0f - x_q.a;
        x_q.y_last = 0;
        initialized = 1;
    }

    int16_t sin, cos;
    int16_t I,Q;
    int32_t mag;
    
    switch(estimation_t.Ls) {
        case MEASUREMENT:
            uint16_t step = estimation_t.counter_Ls / estimation_t.time_div_Ls;
            uint16_t half_time_steps = (estimation_t.counter_Ls / (estimation_t.time_div_Ls / 2)) - (step * 2);

            switch (step)
            {
                case 0:
                    get_angle_from_frequency(hfi_injection_freq[0], &estimation_t.hfi_angle_increment);
                    sin = sin_t(estimation_t.hfi_angle_increment);
                    cos = cos_t(estimation_t.hfi_angle_increment);
                    pHandle_foc->V_dq_q15.d = CLAMP_INT32_TO_INT16(((int32_t)sin * estimation_t.hfi_voltage_q15) >> 15); // inject voltage in d-axis
                    pHandle_foc->V_dq_q15.q = 0;

                    I = CLAMP_INT32_TO_INT16(((int32_t)sin * pHandle_foc->I_dq_q15.d) >> 15); // we are interested in the q-axis current for Ls estimation 
                    Q = CLAMP_INT32_TO_INT16(((int32_t)cos * pHandle_foc->I_dq_q15.d) >> 15);

                    I = slow_iir_filter(I, &x_d);
                    Q = slow_iir_filter(Q, &x_q);

                    I = ((int32_t)I * I) >> 10; // square of the current, scaled back to Q10
                    Q = ((int32_t)Q * Q) >> 10; // square of the current, scaled back to Q10

                    mag = (int32_t)I + (int32_t)Q; // magnitude of the current response in Q10

                    mag = sqrt_fast_uint((mag << 10)); // take square root and scale back to Q10


                    if(half_time_steps != 0){   // take median of first half of the time step to get a stable voltage measurement
                        if(estimation_t.mini_counter_Ls[step]  < 100){      
                            estimation_t.med_current_Ls[step] += (mag << 1); // for Ls estimation we are interested in the q-axis current
                            estimation_t.mini_counter_Ls[step]  ++;
                        }
                    }
                break;
                case 1:
                    get_angle_from_frequency(hfi_injection_freq[1], &estimation_t.hfi_angle_increment);
                    sin = sin_t(estimation_t.hfi_angle_increment);
                    cos = cos_t(estimation_t.hfi_angle_increment);
                    pHandle_foc->V_dq_q15.d = CLAMP_INT32_TO_INT16(((int32_t)sin * estimation_t.hfi_voltage_q15) >> 15); // inject voltage in d-axis
                    pHandle_foc->V_dq_q15.q = 0;

                    I = CLAMP_INT32_TO_INT16(((int32_t)sin * pHandle_foc->I_dq_q15.d) >> 15); // we are interested in the q-axis current for Ls estimation 
                    Q = CLAMP_INT32_TO_INT16(((int32_t)cos * pHandle_foc->I_dq_q15.d) >> 15);

                    I = slow_iir_filter(I, &x_d);
                    Q = slow_iir_filter(Q, &x_q);

                    I = ((int32_t)I * I) >> 10; // square of the current, scaled back to Q10
                    Q = ((int32_t)Q * Q) >> 10; // square of the current, scaled back to Q10

                    mag = (int32_t)I + (int32_t)Q; // magnitude of the current response in Q10

                    mag = sqrt_fast_uint((mag << 10)); // take square root and scale back to Q10


                    if(half_time_steps != 0){   // take median of first half of the time step to get a stable voltage measurement
                        if(estimation_t.mini_counter_Ls[step]  < 100){      
                            estimation_t.med_current_Ls[step] += mag; // for Ls estimation we are interested in the q-axis current
                            estimation_t.mini_counter_Ls[step]  ++;
                        }
                    }
                break;
                case 2:
                    get_angle_from_frequency(hfi_injection_freq[2], &estimation_t.hfi_angle_increment);
                    sin = sin_t(estimation_t.hfi_angle_increment);
                    cos = cos_t(estimation_t.hfi_angle_increment);
                    pHandle_foc->V_dq_q15.d = CLAMP_INT32_TO_INT16(((int32_t)sin * estimation_t.hfi_voltage_q15) >> 15); // inject voltage in d-axis
                    pHandle_foc->V_dq_q15.q = 0;

                    I = CLAMP_INT32_TO_INT16(((int32_t)sin * pHandle_foc->I_dq_q15.d) >> 15); // we are interested in the q-axis current for Ls estimation 
                    Q = CLAMP_INT32_TO_INT16(((int32_t)cos * pHandle_foc->I_dq_q15.d) >> 15);

                    I = slow_iir_filter(I, &x_d);
                    Q = slow_iir_filter(Q, &x_q);

                    I = ((int32_t)I * I) >> 10; // square of the current, scaled back to Q10
                    Q = ((int32_t)Q * Q) >> 10; // square of the current, scaled back to Q10

                    mag = (int32_t)I + (int32_t)Q; // magnitude of the current response in Q10

                    mag = sqrt_fast_uint((mag << 10)); // take square root and scale back to Q10


                    if(half_time_steps != 0){   // take median of first half of the time step to get a stable voltage measurement
                        if(estimation_t.mini_counter_Ls[step]  < 100){      
                            estimation_t.med_current_Ls[step] += mag; // for Ls estimation we are interested in the q-axis current
                            estimation_t.mini_counter_Ls[step]  ++;
                        }
                    }
                break;

                default:
                estimation_t.Ls = CALCULATION;
            }
            estimation_t.counter_Ls++;


        break;
        case CALCULATION:
        float La[3] = {0};
        int16_t V_div_I[3] = {0};
        for(int i = 0; i < 3; i++){
            estimation_t.med_current_Ls[i] = estimation_t.med_current_Ls[i] / (estimation_t.mini_counter_Ls[i]); // take average of the current measurements
            V_div_I[i] = (((int32_t)estimation_t.hfi_voltage_q15 * pHandle_foc->source_voltage))/estimation_t.med_current_Ls[i];
            float x = (((float)V_div_I[i] * (float)V_div_I[i]) / (float)Q10) - (estimation_t.est_Rs * estimation_t.est_Rs); // V^2 / I^2 + Rs^2, scaled back to Q15
            float w = PI_MULTIPLY_2 * (float)hfi_injection_freq[i]; // angular frequency of the injection in rad/s
            La[i] = sqrtf(x) / w;
        }


        debug_induction1 = (float)La[0]; // for debugging: convert back to microhenry
        debug_induction2 = (float)La[1]; // for debugging: convert back to microhenry
        debug_induction3 = (float)La[2]; // for debugging: convert back to microhenry

        estimation_t.est_Ls = (La[0] + La[1] + La[2]) / 3.0f; // take average of the three measurements for better accuracy
        return PROCESS_SUCCESS; // estimation done


        break;
        default:


        break;
    }

    return PROCESS_UNSUCCESS;
}