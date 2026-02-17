#ifndef MOTOR_PARAM_EST_H
#define MOTOR_PARAM_EST_H

#include <stdbool.h>
#include "motor_types.h"
#include "parameter.h"

void MotorParamEst_Init(Motor* m);
bool MotorParamEst_IsDone(const Motor* m);
void MotorParamEst_Service(Motor* m, FOC_HandleTypeDef *foc_values);

/**
 * @brief Zero-intercept linear regression (y = R * x).
 *
 * Computes the least-squares estimate:
 *
 *      R = sum(x_i * y_i) / sum(x_i^2)
 *
 * Intended for resistance estimation (e.g. U = R * I).
 *
 * Fixed-point scaling:
 *  - Inputs x[i], y[i] are in Q10 format
 *        real_value = int_value / 2^10
 *  - Internal products are Q20
 *
 * Since numerator and denominator share identical scaling (Q20),
 * the ratio directly yields the real-valued slope R.
 *
 * Accumulation is performed in 64-bit to avoid overflow.
 * Returns 0.0f if inputs are invalid or denominator is zero.
 *
 * @param[in]  x   Pointer to Q10 independent variable samples
 * @param[in]  y   Pointer to Q10 dependent variable samples
 * @param[in]  n   Number of samples
 *
 * @return Estimated slope R as real-valued float.
 */
float lin_reg(const int32_t *x, const int32_t *y, uint32_t n);


#define RS_ESTIMATION_TIME_PER_STEP (FOC_FREQUENCY/2) // 0.5 seconds per step
#define LS_ESTIMATION_TIME_PER_STEP (FOC_FREQUENCY) // 1 second per step

#endif /* MOTOR_PARAM_EST_H */