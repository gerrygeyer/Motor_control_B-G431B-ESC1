/*
 * parameter estimation.c
 * 
 * 
*/

#include "motor_param_est.h"
#include "foc_math.h"
#include "current_measurement.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "motor_types.h"
#include "parameter.h"
#include "motor_events.h"
#include "motor_hfi.h"
#include "control.h"
#include "motor_inertia_est.h"
#include <math.h>


extern InertiaEstimator_t inertia_est;

static Status resistor_measurement_timing(FOC_HandleTypeDef *pHandle_foc, motor_param_result_t* result);
static Status induction_measurement_timing(FOC_HandleTypeDef *pHandle_foc, motor_param_result_t* result);
static Status backEMF_measurement_timing(Motor *m, FOC_HandleTypeDef *pHandle_foc, motor_param_result_t *result);

static bool pidm_est_j_active;
static uint32_t pidm_est_j_counter;

static void pidm_reset_j_estimation(void)
{
    pidm_est_j_active = false;
    pidm_est_j_counter = 0u;
}

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
    pidm_reset_j_estimation();
}
/* Info: this function are triggert by sm machine in while loop*/
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


    estimation_t.counter_Ke = 0;
    estimation_t.time_div_Ke = KE_ESTIMATION_TIME_PER_STEP;
    estimation_t.Ke = MEASUREMENT;
    estimation_t.counter_Ke_measurement = 0;
    estimation_t.med_Vq15 = 0;
    estimation_t.med_Iq15 = 0;
    estimation_t.med_speed_q15 = 0;


    m->pidm_init_done_flag = true; 
}

bool MotorParamEst_IsDone(const Motor* m)
{
    if (m == NULL) return false;
    return (m->pidm_state == PIDM_DONE) && (m->pidm_result.valid == true);
}

void MotorParamEst_Service(Motor* m, Control_Loops *ctrl, FOC_HandleTypeDef *foc_values)
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
            
            foc_values->current_ff_flag = false; // deactivate feed forward during estimation; because we assume that the known engine parameters are incorrect
            
            m->pidm_start_request_flag = true; // hey, here we 'explicit' start yet :)

            if(m->pidm_start_request_flag){
                m->pidm_start_request_flag = false;
                m->pidm_state = PIDM_START;
            }
            break;

        case PIDM_START:

            /* TODO: initialize injections / observers */
            m->pidm_state = PIDM_EST_R;
            break;

        case PIDM_EST_R:
            
            /* Estimate stator resistance Rs */
            foc_values->foc_mode = FOC_CURRENT_CONTROL;
            if (resistor_measurement_timing(foc_values, &m->pidm_result) == PROCESS_SUCCESS) {

                foc_values->V_abc_q15 = (abc_16t){0,0,0}; // set voltage to zero after estimation is done
                m->pidm_state = PIDM_EST_L;
            }

            break;

        case PIDM_EST_L:
            /* Estimate inductance Ls */
            foc_values->foc_mode = FOC_HFI;
            if(induction_measurement_timing(foc_values, &m->pidm_result) == PROCESS_SUCCESS) {
            foc_values->V_abc_q15 = (abc_16t){0,0,0};
            m->pidm_state = PIDM_EST_Ke;

            // update control parameters with the new estimated values
            ctrl->motor_params.Ls               = m->pidm_result.Ls_h;
            ctrl->motor_params.Rs               = m->pidm_result.Rs_ohm;
            ctrl->alpha                         = (int16_t)(ESTIMATION_PI_IP_ALPHA * (float)Q15);
            ctrl->motor_params.bandwidth_speed  = ESTIMATION_BANDWIDTH_SPEED;
            ctrl->motor_params.cutoff_freq_div  = ESTIMATION_CUTOFF_FREQ_DIV;
            calculate_PI_parameter(ctrl);

            }
            break;

        case PIDM_EST_Ke:
            /* Estimate back-EMF constant Ke */
            foc_values->foc_mode = FOC_CLOSELOOP;

            Status backEMF_status = backEMF_measurement_timing(m,foc_values, &m->pidm_result);

            if(backEMF_status == PROCESS_ERROR) {
                foc_values->V_abc_q15 = (abc_16t){0,0,0};
                m->pidm_state = PIDM_ERROR;
            }
            if(backEMF_status == PROCESS_SUCCESS) {
                foc_values->V_abc_q15 = (abc_16t){0,0,0};
                ctrl->motor_params.Ke = m->pidm_result.Ke_vrpm;
                m->pidm_state = PIDM_EST_J;
            }
            break;

        case PIDM_EST_J:
            /* Estimate inertia J */
            foc_values->foc_mode = FOC_CLOSELOOP;

            if (!pidm_est_j_active) {
                pidm_est_j_active = true;
                pidm_est_j_counter = 0u;
                InertiaEstimator_Reset(&inertia_est, 1000.0f);
                InertiaEstimator_Init(&inertia_est,
                        0.995f,
                        1000.0f,
                        ((3.0f/2.0f)*((float)m->pidm_result.Ke_vrpm * 60.0f / PI_MULTIPLY_2)),
                        1e-6f,
                        1e-1f);
                InertiaEstimator_Enable(&inertia_est, true);
            }

            if (abs(foc_values->I_dq_q15.q) > MAX_ESTIMATION_CURRENT) {
                m->speed_ref = 0;
                pidm_reset_j_estimation();
                m->pidm_state = PIDM_ERROR;
                break;
            }

            {
                const uint32_t ramp_ticks = FOC_FREQUENCY;
                const uint32_t timeout_ticks = 15u * FOC_FREQUENCY;
                const int16_t speed_min_rpm = 400;
                const int16_t speed_max_rpm = 1000;
                const int32_t speed_span_rpm = (int32_t)speed_max_rpm - (int32_t)speed_min_rpm;
                uint32_t ramp_pos = pidm_est_j_counter % (2u * ramp_ticks);

                if (ramp_pos < ramp_ticks) {
                    m->speed_ref = (int16_t)(speed_min_rpm + (int16_t)((speed_span_rpm * (int32_t)ramp_pos) / (int32_t)ramp_ticks));
                } else {
                    ramp_pos -= ramp_ticks;
                    m->speed_ref = (int16_t)(speed_max_rpm - (int16_t)((speed_span_rpm * (int32_t)ramp_pos) / (int32_t)ramp_ticks));
                }

                if (InertiaEstimator_IsValid(&inertia_est) && (inertia_est.update_count >= J_ESTIMATION_UPDATE_COUNT)) {
                    m->pidm_result.J_kgm2 = InertiaEstimator_GetJ(&inertia_est);
                    m->pidm_result.B_nmsrad = InertiaEstimator_GetB(&inertia_est);
                    m->speed_ref = 0;
                    pidm_reset_j_estimation();
                    m->pidm_result.valid = true;
                    m->pidm_state = PIDM_DONE;
                    break;
                }

                pidm_est_j_counter++;
                if (pidm_est_j_counter >= timeout_ticks) {
                    m->speed_ref = 0;
                    pidm_reset_j_estimation();
                    m->pidm_state = PIDM_ERROR;
                    break;
                }
            }

            break;

        case PIDM_DONE:
            ctrl->motor_params.J = m->pidm_result.J_kgm2;
            ctrl->motor_params.B = m->pidm_result.B_nmsrad;
            ctrl->alpha = (int16_t)(PI_IP_ALPHA * (float)Q15);
            ctrl->motor_params.bandwidth_speed = BANDWIDTH_SPEED;
            ctrl->motor_params.cutoff_freq_div = CUTOFFF_FREQU_DIV;
            calculate_PI_parameter(ctrl);
            btn_stop_edge = true;           
            foc_values->current_ff_flag = true; // reactivate feed forward after estimation is done;
            /* Hold results until user leaves ST_PARAMETER_ID or starts again */
            break;

        case PIDM_ERROR:
            // error same as default case
        default:
            /* Keep motor safe */
            btn_stop_edge = true;
            foc_values->current_ff_flag = true; 
            /* TODO: disable injections / set outputs safe */
            break;
    }
}




static Status resistor_measurement_timing(FOC_HandleTypeDef *pHandle_foc, motor_param_result_t*result){

    switch (estimation_t.Rs) {
        case MEASUREMENT: {

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

            }
            break;
        }  
        estimation_t.counter_Rs++;
        break;
        case CALCULATION:
        pHandle_foc->I_ref_q15.q = 0;
        pHandle_foc->I_ref_q15.d = 0;
        estimation_t.est_Rs = lin_reg(estimation_t.current_injection_q15, estimation_t.med_voltage_Rs, 6);
        estimation_t.est_Rs = estimation_t.est_Rs * 0.5; 
        estimation_t.Rs = RESULT;
    }
        break;

        case RESULT:
        result->Rs_ohm = estimation_t.est_Rs;
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

int32_t mag;
static Status induction_measurement_timing(FOC_HandleTypeDef *pHandle_foc, motor_param_result_t*result){

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
    // int32_t mag;
    
    switch(estimation_t.Ls) {
        case MEASUREMENT: {
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
                    
                    mag = (mag << 1);

                    if(half_time_steps != 0){   // take median of first half of the time step to get a stable voltage measurement
                        if(estimation_t.mini_counter_Ls[step]  < 100){      
                            estimation_t.med_current_Ls[step] += mag; 
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
                    mag = (mag << 1);

                    if(half_time_steps != 0){   // take median of first half of the time step to get a stable voltage measurement
                        if(estimation_t.mini_counter_Ls[step]  < 100){      
                            estimation_t.med_current_Ls[step] += mag;
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

                    mag = (mag << 1);


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

        }
        break;
        case CALCULATION: {
        float La[3] = {0};
        int16_t V_div_I[3] = {0};
        for(int i = 0; i < 3; i++){
            
            estimation_t.med_current_Ls[i] = estimation_t.med_current_Ls[i] / (estimation_t.mini_counter_Ls[i]); // take average of the current measurements
            V_div_I[i] = ((((int32_t)estimation_t.hfi_voltage_q15 * pHandle_foc->source_voltage))/(estimation_t.med_current_Ls[i]));
            float x = (((float)V_div_I[i]/(float)Q16) * ((float)V_div_I[i] / (float)Q16)) - (estimation_t.est_Rs * estimation_t.est_Rs); // V^2 / I^2 + Rs^2, scaled back to Q15
            float w = PI_MULTIPLY_2 * (float)hfi_injection_freq[i]; // angular frequency of the injection in rad/s
            if(x < 0){
                La[i] = -1; // if the value under the square root is negative, we set the inductance to zero to avoid NaN values. This can happen if the current response is very small, which can be the case for very low inductance or high resistance.
            }else{
                La[i] = sqrtf(x) / w;
            }
        }

        estimation_t.med_current_Ls[0];
        estimation_t.med_current_Ls[1];
        estimation_t.med_current_Ls[2];

        // debug_induction1 = (float)La[0]; // for debugging: convert back to microhenry
        // debug_induction2 = (float)La[1]; // for debugging: convert back to microhenry
        // debug_induction3 = (float)La[2]; // for debugging: convert back to microhenry

        uint8_t valid_measurements = 3;
        for(uint8_t i = 0; i < 3; i++){
            if(La[i] < 0){
                La[i] = 0; // if the inductance is negative, we set it to zero to avoid NaN values in the final result. This can happen if the current response is very small, which can be the case for very low inductance or high resistance.
                valid_measurements--;
            }
        }
        if(valid_measurements > 0){
            estimation_t.est_Ls = (La[0] + La[1] + La[2]) / (float)valid_measurements;
        }else{
            estimation_t.est_Ls = 0; // if all measurements are invalid, we set the inductance to zero
            return PROCESS_UNSUCCESS;
        }
        estimation_t.Ls = RESULT;

        break;
        case RESULT:

        result->Ls_h = estimation_t.est_Ls;

        return PROCESS_SUCCESS; // estimation done
        }
        break;
        default:
        break;
    }

    return PROCESS_UNSUCCESS;
}

static Status backEMF_measurement_timing(Motor *m, FOC_HandleTypeDef *pHandle_foc, motor_param_result_t *result) {

    switch (estimation_t.Ke) {
        case MEASUREMENT: {
            uint16_t step = estimation_t.counter_Ke / estimation_t.time_div_Ke;

            m->speed_ref = 500; 

            if (abs(pHandle_foc->I_ref_q15.q) > Q14) { // need to run without load, so this case should not happen, if it does, we abort the estimation to avoid damage to the motor
                m->speed_ref = 0;
                return PROCESS_ERROR;
            }

            switch (step) {
                case 0:
                case 1:
                    break;
                default:
                    if (estimation_t.counter_Ke_measurement < FOC_FREQUENCY) {
                        estimation_t.counter_Ke_measurement++;
                        estimation_t.med_Iq15 += abs(pHandle_foc->I_dq_q15.q); // we are interested in the q-axis current for Ke estimation
                        estimation_t.med_Vq15 += abs(pHandle_foc->V_dq_q15.q); // we are interested in the q-axis voltage for Ke estimation
                        estimation_t.med_speed_q15 += abs(pHandle_foc->speed_q15);
                    } else {
                        estimation_t.med_Iq15 = estimation_t.med_Iq15 / FOC_FREQUENCY; // take average of the current measurements
                        estimation_t.med_Vq15 = estimation_t.med_Vq15 / FOC_FREQUENCY; // take average of the voltage measurements
                        estimation_t.med_speed_q15 = estimation_t.med_speed_q15 / FOC_FREQUENCY; // take average of the speed measurements
                        estimation_t.Ke = CALCULATION;
                    }
                    break;
            }
            estimation_t.counter_Ke++;
        }
        break;
        case CALCULATION: {
            float Vq = ((float)estimation_t.med_Vq15 * (float)pHandle_foc->source_voltage) / (float)Q26; // convert back to real voltage in V (Vq is in Q10, and we need to divide by 2 to get the real voltage (Vdc/2) -> Q11 ;  than, divide by Q15 to get the real voltage in V) 
            float Iq = ((float)estimation_t.med_Iq15) / (float)Q10; // convert back to real current in A
            float speed_rpm = ((float)estimation_t.med_speed_q15) * (float)MAX_SPEED / (float)Q15;
            if(fabs(speed_rpm) < 0.1f) { // avoid division by zero and very low speeds which can lead to very high Ke estimates due to noise
                return PROCESS_ERROR;
            }
            estimation_t.est_Ke = (float)PMSM_POLEPAIR * (Vq - estimation_t.est_Rs * Iq) / speed_rpm; // Ke = V / omega
            estimation_t.Ke = RESULT;
        }
        break;
        case RESULT:
            result->Ke_vrpm = estimation_t.est_Ke;
            return PROCESS_SUCCESS; // estimation done
            break;
    }
    
    return PROCESS_UNSUCCESS;
}
