/*
 * math.h
 *
 *  Created on: Jul 2, 2024
 *      Author: xboxg
 */

#ifndef INC_FOC_MATH_H_
#define INC_FOC_MATH_H_

#include <stdint.h>
#include "parameter.h"


// pre-calculation
#define SQRT3 			1.7320508075688772935274463415059
#define DIV_1_SQRT3		0.577350269189626
#define DIV_1_SQRT3_Q15	18919
#define SQRT3_Q14		28378 // its in q14 because q15 is > int16_max
#define PI_VAL			3.1415926535897931
#define PI_MULTIPLY_2	6.2831853071795865
#define DIV_180_PI		57.295779513082320876798154814105
#define DIV_PI_3 		1.0471975511965977461542144610932
#define DIV_PI_2		1.5707963267948966192313216916398
#define SIN_DIV_PI_3	0.866025403784439
#define DIV_1_LN_2		1.442695040888963407
#define DIV_2_SQRT3		1.1547005383792515290182975610039  // 2/sqrt(3)
#define RPM2RAD			0.104719755119660f


// definition
#define UINT12_MAX_VALUE	0xFFF		// 4096
#define UINT16_MAX_VALUE	0xFFFF		//65535
#define UINT32_MAX_VALUE	0xFFFFFFFF //4.294.967.295
#define INT16_MAX_VALUE		0x7FFF
#define INT16_HALF_VALUE	0x3FFF
#define UINT12_HALF_VALUE	0x7FF		// 2047

#define IIR_PL_RAD			10000.0
#define IIR_LP_FC			(uint32_t)(IIR_PL_RAD/(2 * 3.1415))
#define IIR_LP_CURRENT_A 	((PI_MULTIPLY_2 * FOC_TS * IIR_LP_FC) / ((PI_MULTIPLY_2 * FOC_TS * IIR_LP_FC) + 1))

#define IIR_vPL_RPM			10000	// RPM
#define IIR_vLP_FC			((IIR_vPL_RPM * PMSM_POLEPAIR) / (60 * PI_MULTIPLY_2))
#define IIR_vLP_CURRENT_A 	((PI_MULTIPLY_2 * FOC_TS * IIR_vLP_FC) / ((PI_MULTIPLY_2 * FOC_TS * IIR_vLP_FC) + 1))

#define IIR_ZERO_A			((PI_MULTIPLY_2 * FOC_TS * 1.0f)/(PI_MULTIPLY_2 * FOC_TS * 1.0f + 1.0f))
#define RPM_TO_RAD_S		0.104719755119660f

#define Q10					1024
#define Q11					2048
#define Q12					4096
#define Q13					8192
#define Q14					16384
#define Q15					32768
#define Q16					65536
#define Q18					262144
#define Q19					524288
#define Q20					1048576
#define Q22					4194304
#define Q29					536870912
#define Q30					1073741824

#define RAD2RPMQ15			3431
#define DIV_2_SQRT3Q14		18919

// shortcuts (new)
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define CLAMP_INT32_TO_INT16(x) ((x) > INT16_MAX ? INT16_MAX : ((x) < INT16_MIN ? INT16_MIN : (int16_t)(x)))

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))


typedef struct{
	dq_f old;
	dq_f new;
}memory_dq;

typedef struct{
	dq_32t old;
	dq_32t new;
}memory_dq_t;

typedef struct{
	alphabeta_f old;
	alphabeta_f new;
}memory_alpha_beta;


void init_math(void);

float getSineValue_360(int16_t y);
float getCosValue_360(int16_t y);
int16_t get_atan_value(int16_t d, int16_t q);

dq_f circle_limiter_maxVoltage(dq_f v_12);
dq_f IIR_LP_filter_dq(dq_f I);
alphabeta_f IIR_LP_filter_alpha_beta(alphabeta_f I);
alphabeta_f IIR_LP_filter_observer_alpha_beta(alphabeta_f I, float a);
int16_t IIR_LP_filter_Speed(int16_t input);
int16_t IIR_LP_filter_Speed_f(int16_t input);
float limit_f(float input, float max_value);
float eucl_norm2_approx(float x, float y);

float IIR_LP_filter_Iq_set(float iq);
float sigmoid_function_approx(float x);
float signum_function(float x);
int16_t atan2_approx(float y, float x);

int16_t getAngle(int16_t x, int16_t y);
void test(int16_t x, int16_t y);


int16_t sin_t(int16_t y);
int16_t cos_t(int16_t y);


/**
 * @brief       Fast approximation of the integer square root.
 *
 * @details     Computes an approximate square root of an unsigned 32-bit integer using
 *              an initial guess based on bit position and 3 Newton-Raphson iterations.
 *              Final result is corrected if necessary to ensure \f$x^2 \leq n\f$.
 *
 * @param       n       Unsigned 32-bit input value.
 *
 * @return      Approximate floor of \f$\sqrt{n}\f$ as unsigned 32-bit integer.
 *
 * @note        Accurate up to ±1. Performs well for embedded applications with tight timing.
 * @warning     For n = 0, the result is 0. No protection against division by zero needed.
 */
uint32_t sqrt_fast_uint(uint32_t n);

/**
 * @brief       Limits the magnitude of a dq-vector to a given maximum using Q15 fixed-point arithmetic.
 *
 * @details     If the magnitude of the input vector `Vdq` exceeds `max_output`, it is scaled down to fit
 *              within the circle of radius `max_output`. Otherwise, the vector remains unchanged.
 *
 * @param       Vdq             Input vector in Q15 format (components in range −32768 … +32767).
 * @param       max_output      Maximum allowed magnitude (Q15), e.g., 32767 for full scale.
 *
 * @return      Scaled dq-vector with magnitude ≤ `max_output`, in Q15 format.
 *
 * @note
 * - Uses `sqrt_fast_uint()` for efficient magnitude calculation.
 * - Applies fixed-point scaling: \f$ \text{scale} = \frac{\text{max} \ll 15}{|v|} \f$
 * - Returns [0, 0] if input vector has zero length.
 *
 * @see         sqrt_fast_uint(), CLAMP_INT32_TO_INT16
 */
dq_t circle_limitation_Q15(dq_t Vdq, const uint32_t max_output);

#endif /* INC_FOC_MATH_H_ */
