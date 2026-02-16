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
#define SQRT3 			1.7320508075688772935274463415059f
#define DIV_1_SQRT3		0.577350269189626f
#define DIV_1_SQRT3_Q15	18919
#define SQRT3_Q14		28378 // its in q14 because q15 is > int16_max
#define PI_VAL			3.1415926535897931f
#define PI_MULTIPLY_2	6.2831853071795865f
#define DIV_180_PI		57.295779513082320876798154814105f
#define DIV_PI_3 		1.0471975511965977461542144610932f
#define DIV_PI_2		1.5707963267948966192313216916398f
#define SIN_DIV_PI_3	0.866025403784439f
#define DIV_1_LN_2		1.442695040888963407f
#define DIV_2_SQRT3		1.1547005383792515290182975610039f  // 2/sqrt(3)
#define RPM2RAD			0.104719755119660f


// definition
#define UINT12_MAX_VALUE	0xFFF		// 4096
#define UINT16_MAX_VALUE	0xFFFF		//65535
#define UINT32_MAX_VALUE	0xFFFFFFFF //4.294.967.295
#define INT16_MAX_VALUE		0x7FFF
#define INT16_HALF_VALUE	0x3FFF
#define UINT12_HALF_VALUE	0x7FF		// 2047

#define RPM_TO_RAD_S		0.104719755119660f

#define Q5					(32-1)
#define Q10					(1024-1)
#define Q11					(2048-1)
#define Q12					(4096-1)
#define Q13					(8192-1)
#define Q14					(16384-1)
#define Q15					(32768-1)
#define Q16					(65536-1)
#define Q18					(262144-1)
#define Q19					(524288-1)
#define Q20					(1048576-1)
#define Q22					(4194304-1)
#define Q29					(536870912-1)
#define Q30					(1073741824-1)

#define RAD2RPMQ15			3431
#define DIV_2_SQRT3Q14		18919

// shortcuts (new)
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define CLAMP_INT32_TO_INT16(x) ((x) > INT16_MAX ? INT16_MAX : ((x) < INT16_MIN ? INT16_MIN : (int16_t)(x)))

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))



typedef struct{
	dq_32t old;
	dq_32t new;
}memory_dq_t;



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

// linear regression for parameter estimation
ab_t lin_reg(const int32_t *x, const int32_t *y, uint32_t n);

#endif /* INC_FOC_MATH_H_ */
