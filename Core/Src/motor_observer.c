#include "motor_observer.h"
#include "foc_math.h"
#include "parameter.h"

sliding_mode_t sliding_mode;

static int16_t sigmoid_q10(int16_t x);
static void sliding_mode_observer(FOC_HandleTypeDef *pHandle_foc);

void init_sliding_mode(void){
    
    sliding_mode.Rs = get_q_format(PMSM_RS_OHM);
    float Ts_over_LS_f = 1.0f / ((float)PMSM_LS * (float)FOC_FREQUENCY);
    sliding_mode.Ts_over_Ls = get_q_format(Ts_over_LS_f);
    sliding_mode.k_gain = 10; // TEST

    if(sliding_mode.Rs.q == Qerror || sliding_mode.Ts_over_Ls.q == Qerror){
        sliding_mode.observer_initialized = OFF;
    }else{
        sliding_mode.observer_initialized = ON;
    }
}

void init_observer(void){

    init_sliding_mode();

}
void trigger_observer(FOC_HandleTypeDef *pHandle_foc){

    if(sliding_mode.observer_initialized == ON){
        sliding_mode_observer(pHandle_foc);
    }
}




static void sliding_mode_observer(FOC_HandleTypeDef *pHandle_foc){

    sliding_mode.I_dot_est.alpha    = -((int32_t)sliding_mode.Rs.value * sliding_mode.I_est.alpha) >> (sliding_mode.Rs.q);  // current in Q10
    sliding_mode.I_dot_est.beta     = -((int32_t)sliding_mode.Rs.value * sliding_mode.I_est.beta) >> (sliding_mode.Rs.q);    // current in Q10

    sliding_mode.I_dot_est.alpha    += ((int32_t)pHandle_foc->V_alph_bet_q15.alpha * pHandle_foc->source_voltage) >> 15;   // add voltage in Q10
    sliding_mode.I_dot_est.beta     += ((int32_t)pHandle_foc->V_alph_bet_q15.beta * pHandle_foc->source_voltage) >> 15;     // add voltage in Q10

    sliding_mode.I_dot_est.alpha    -= sliding_mode.kH.alpha;
    sliding_mode.I_dot_est.beta     -= sliding_mode.kH.beta;

    sliding_mode.I_est.alpha        += (sliding_mode.I_dot_est.alpha * sliding_mode.Ts_over_Ls.value) >> sliding_mode.Ts_over_Ls.q;
    sliding_mode.I_est.beta         += (sliding_mode.I_dot_est.beta * sliding_mode.Ts_over_Ls.value) >> sliding_mode.Ts_over_Ls.q;

    sliding_mode.I_est.alpha        = CLAMP(sliding_mode.I_est.alpha,-Q15,Q15);
    sliding_mode.I_est.beta         = CLAMP(sliding_mode.I_est.beta,-Q15,Q15);

    alphabeta32_t x;
    x.alpha                         = CLAMP_INT32_TO_INT16((int32_t)sliding_mode.alpha_gain * ((int32_t)sliding_mode.I_est.alpha - pHandle_foc->I_alph_bet_q15.alpha));
    x.beta                          = CLAMP_INT32_TO_INT16((int32_t)sliding_mode.alpha_gain * ((int32_t)sliding_mode.I_est.beta - pHandle_foc->I_alph_bet_q15.beta));
    
    alphabeta32_t alphabeta_sigmoid;
    alphabeta_sigmoid.alpha         = sigmoid_q10(x.alpha);
    alphabeta_sigmoid.beta          = sigmoid_q10(x.beta);

    sliding_mode.kH.alpha           = CLAMP_INT32_TO_INT16(alphabeta_sigmoid.alpha * (int32_t)sliding_mode.k_gain);
    sliding_mode.kH.beta            = CLAMP_INT32_TO_INT16(alphabeta_sigmoid.beta * (int32_t)sliding_mode.k_gain);

    sliding_mode.theta_est          = fast_atan2_u16(&sliding_mode.kH);

}



/**
 * @brief Sigmoid approximation using a 256-point LUT with linear interpolation.
 *
 * Input is given in Q10 format over the range [-5, 5].
 * Output corresponds to the true sigmoid function scaled to Q15.
 *
 * @param x Input value in Q10.
 * @return Approximated sigmoid output.
 */
static int16_t sigmoid_q10(int16_t x)
{
    if (x >= 5120)  return sigmoid_lut[255];
    if (x <= -5120) return (int16_t)(-sigmoid_lut[255]);

    int16_t abs_x = (x >= 0) ? x : (int16_t)(-x);

    uint32_t pos_q8 = ((uint32_t)abs_x * 51u + 2u) >> 2;
    uint16_t idx    = (uint16_t)(pos_q8 >> 8);
    uint16_t frac   = (uint16_t)(pos_q8 & 0xFFu);

    int32_t y;

    if (idx >= 255u)
    {
        y = sigmoid_lut[255];
    }
    else
    {
        int32_t y0 = sigmoid_lut[idx];
        int32_t y1 = sigmoid_lut[idx + 1];
        y = y0 + (((y1 - y0) * (int32_t)frac + 128) >> 8);
    }

    return (x >= 0) ? (int16_t)y : (int16_t)(-y);
}