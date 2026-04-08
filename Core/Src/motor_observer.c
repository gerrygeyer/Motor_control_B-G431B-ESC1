#include "motor_observer.h"
#include "foc_math.h"
#include "parameter.h"

sliding_mode_t sliding_mode;

static int16_t sigmoid_q10(int16_t x);

void init_sliding_mode(void){
    
    sliding_mode.Rs = get_q_format(PMSM_RS_OHM);
    float delta_t_f = 1.0f / ((float)PMSM_LS * (float)FOC_FREQUENCY);
    sliding_mode.delta_t = get_q_format(delta_t_f);
    sliding_mode.k_gain = 10; // TEST
}

void init_observer(void){

    init_sliding_mode();

}




void sliding_mode_observer(FOC_HandleTypeDef *pHandle_foc){

    sliding_mode.I_dot_est.alpha = ((int32_t)sliding_mode.Rs.value * sliding_mode.I_est.alpha) >> (sliding_mode.Rs.q);  // current in Q10
    sliding_mode.I_dot_est.beta = ((int32_t)sliding_mode.Rs.value * sliding_mode.I_est.beta) >> (sliding_mode.Rs.q);    // current in Q10

    sliding_mode.I_dot_est.alpha += ((int32_t)pHandle_foc->V_alph_bet_q15.alpha * pHandle_foc->source_voltage) >> 15;   // add voltage in Q10
    sliding_mode.I_dot_est.beta += ((int32_t)pHandle_foc->V_alph_bet_q15.beta * pHandle_foc->source_voltage) >> 15;     // add voltage in Q10

    sliding_mode.I_dot_est.alpha += sliding_mode.kH.alpha;
    sliding_mode.I_dot_est.beta += sliding_mode.kH.beta;

    sliding_mode.I_est.alpha += (sliding_mode.I_dot_est.alpha * sliding_mode.delta_t.value) >> sliding_mode.delta_t.q;
    sliding_mode.I_est.beta += (sliding_mode.I_dot_est.beta * sliding_mode.delta_t.value) >> sliding_mode.delta_t.q;

    sliding_mode.I_est.alpha = CLAMP(sliding_mode.I_est.alpha,-Q15,Q15);
    sliding_mode.I_est.beta = CLAMP(sliding_mode.I_est.beta,-Q15,Q15);

    sliding_mode.kH.alpha = sigmoid_q10(CLAMP_INT32_TO_INT16((int32_t)sliding_mode.k_gain * ((int32_t)sliding_mode.I_est.alpha - pHandle_foc->I_alph_bet_q15.alpha)));
    sliding_mode.kH.beta = sigmoid_q10(CLAMP_INT32_TO_INT16((int32_t)sliding_mode.k_gain * ((int32_t)sliding_mode.I_est.beta - pHandle_foc->I_alph_bet_q15.beta)));
    

}







/**
 * @brief 256-point lookup table for sigmoid function
 */
const int16_t sigmoid_lut[256] = {
     0,    321,    642,    963,   1284,   1605,   1925,   2245,
  2565,   2884,   3202,   3520,   3837,   4154,   4469,   4784,
  5098,   5411,   5723,   6034,   6344,   6652,   6960,   7266,
  7571,   7874,   8176,   8477,   8776,   9073,   9369,   9663,
  9955,  10246,  10535,  10822,  11107,  11391,  11672,  11952,
 12229,  12505,  12778,  13049,  13319,  13586,  13851,  14114,
 14374,  14632,  14888,  15142,  15394,  15643,  15890,  16134,
 16376,  16616,  16854,  17089,  17321,  17552,  17780,  18005,
 18228,  18449,  18667,  18883,  19096,  19307,  19515,  19722,
 19925,  20126,  20325,  20522,  20716,  20907,  21097,  21284,
 21468,  21650,  21830,  22008,  22183,  22356,  22526,  22695,
 22861,  23024,  23186,  23345,  23502,  23657,  23810,  23960,
 24109,  24255,  24399,  24541,  24681,  24819,  24955,  25089,
 25221,  25351,  25479,  25605,  25729,  25851,  25972,  26090,
 26207,  26322,  26435,  26546,  26656,  26763,  26869,  26974,
 27077,  27178,  27277,  27375,  27471,  27566,  27659,  27750,
 27841,  27929,  28016,  28102,  28186,  28269,  28351,  28431,
 28509,  28587,  28663,  28738,  28811,  28883,  28954,  29024,
 29093,  29160,  29226,  29292,  29356,  29418,  29480,  29541,
 29600,  29659,  29717,  29773,  29829,  29883,  29937,  29989,
 30041,  30092,  30142,  30191,  30239,  30286,  30332,  30378,
 30423,  30467,  30510,  30552,  30594,  30634,  30675,  30714,
 30753,  30790,  30828,  30864,  30900,  30935,  30970,  31004,
 31037,  31070,  31102,  31134,  31165,  31195,  31225,  31254,
 31283,  31311,  31339,  31366,  31392,  31418,  31444,  31469,
 31494,  31518,  31542,  31565,  31588,  31611,  31633,  31654,
 31676,  31697,  31717,  31737,  31757,  31776,  31795,  31814,
 31832,  31850,  31867,  31885,  31902,  31918,  31934,  31950,
 31966,  31981,  31996,  32011,  32026,  32040,  32054,  32068,
 32081,  32094,  32107,  32120,  32132,  32145,  32156,  32168,
 32180,  32191,  32202,  32213,  32224,  32234,  32244,  32254,
 32264,  32274,  32284,  32293,  32302,  32311,  32320,  32328

};

/**
 * @brief Sigmoid approximation using a 256-point LUT with linear interpolation.
 *
 * Input is given in Q10 format over the range [-5, 5].
 * Output is scaled to [-32767, 32767].
 *
 * @param x Input value in Q10.
 * @return Approximated sigmoid output.
 */
static int16_t sigmoid_q10(int16_t x)
{
    if (x >= 5120)  return sigmoid_lut[255];     // +5.0 in Q10
    if (x <= -5120) return (int16_t)(-sigmoid_lut[255]); // -5.0 in Q10

    int16_t abs_x = (x >= 0) ? x : (int16_t)(-x);

    /*
     * Map abs_x from [0, 5120] to a fractional LUT position in Q8:
     *
     * pos_q8 = abs_x * 255 * 256 / 5120
     *        = abs_x * 51 / 4
     *
     * idx  = integer LUT index
     * frac = fractional part in [0..255]
     */
    uint32_t pos_q8 = ((uint32_t)abs_x * 51u + 2u) >> 2;   // rounded
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

        /* linear interpolation with rounding */
        y = y0 + (((y1 - y0) * (int32_t)frac + 128) >> 8);
    }

    return (x >= 0) ? (int16_t)y : (int16_t)(-y);
}