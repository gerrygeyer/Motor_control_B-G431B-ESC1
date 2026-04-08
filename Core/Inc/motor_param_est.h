#ifndef MOTOR_PARAM_EST_H
#define MOTOR_PARAM_EST_H

#include <stdbool.h>
#include "motor_types.h"
#include "parameter.h"

void MotorParamEst_Init(Motor* m);
bool MotorParamEst_IsDone(const Motor* m);
void MotorParamEst_Service(Motor* m, Control_Loops *ctrl, FOC_HandleTypeDef *foc_values);

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
#define LS_ESTIMATION_TIME_PER_STEP (FOC_FREQUENCY*2) // 2 seconds per step
#define KE_ESTIMATION_TIME_PER_STEP (FOC_FREQUENCY*2) // 2 seconds per step

#define MAX_ESTIMATION_CURRENT      (2 << 10)       // 2A in Q10 format, for current injection during estimation    
#define J_ESTIMATION_UPDATE_COUNT   (1000u)         // number of iteratrions before we consider the inertia estimation to be valid

#define ESTIMATION_BANDWIDTH_SPEED  20.0f          // desired bandwidth for the speed control loop in Hz, used to calculate the PI parameters after estimation
#define ESTIMATION_CUTOFF_FREQ_DIV  10.0f          // cutoff frequency divider (min. 5)
#define PI_IP_ALPHA                 0.2f           // blending factor for PI to IP; lower values mean more IP and less PI, higher values mean more PI and less IP

#endif /* MOTOR_PARAM_EST_H */