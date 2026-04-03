/*
 * motor_inertia_est.c
 *
 *  Created on: Dec 5, 2024
 */

#ifndef INC_MOTOR_INERTIA_EST_H_
#define INC_MOTOR_INERTIA_EST_H_


#include <stdint.h>
#include <stdbool.h>

#define INERTIA_ESTIMATOR_EPS (1e-12f)

typedef struct
{
    float theta[3];          // [theta1, theta2, theta3]
    float P[3][3];           // covariance matrix
    float lambda;            // forgetting factor

    float kt;                // torque constant [Nm/A]

    float j_hat;             // estimated inertia [kg m^2]
    float b_hat;             // estimated viscous friction [Nms/rad]
    float load_over_j_hat;   // = T_L / J  (with sign according to model)

    float j_min;             // plausibility lower bound
    float j_max;             // plausibility upper bound

    float iq_min;            // minimum |Iq| for update
    float alpha_min;         // minimum |alpha| for update

    float last_error;        // model error
    float last_y_hat;        // predicted acceleration
    float last_den;          // RLS denominator

    uint32_t update_count;
    bool initialized;
    bool enabled;
    bool valid;

    bool start_flag;
} InertiaEstimator_t;

/**
 * @brief Initialize inertia estimator.
 *
 * @param est        Pointer to estimator object
 * @param lambda     Forgetting factor, typically 0.995f ... 1.0f
 * @param p0         Initial covariance diagonal value, e.g. 1000.0f
 * @param kt         Torque constant [Nm/A]
 * @param j_min      Minimum plausible inertia [kg m^2]
 * @param j_max      Maximum plausible inertia [kg m^2]
 */
void InertiaEstimator_Init(InertiaEstimator_t *est,
                           float lambda,
                           float p0,
                           float kt,
                           float j_min,
                           float j_max);

/**
 * @brief Reset estimator state while keeping configuration.
 *
 * @param est        Pointer to estimator object
 * @param p0         New initial covariance diagonal value
 */
void InertiaEstimator_Reset(InertiaEstimator_t *est, float p0);

/**
 * @brief Enable or disable estimator updates.
 *
 * @param est        Pointer to estimator object
 * @param enable     true = enable updates, false = disable updates
 */
void InertiaEstimator_Enable(InertiaEstimator_t *est, bool enable);

/**
 * @brief Update estimator with one new sample.
 *
 * Model:
 *   alpha = theta1 * iq + theta2 * omega + theta3
 *
 * with:
 *   theta1 = Kt / J
 *   theta2 = -B / J
 *   theta3 = -T_L / J
 *
 * @param est        Pointer to estimator object
 * @param iq         q-axis current [A]
 * @param omega      mechanical angular speed [rad/s]
 * @param alpha      mechanical angular acceleration [rad/s^2]
 */
void InertiaEstimator_Update(InertiaEstimator_t *est,
                             float iq,
                             float omega,
                             float alpha);

/**
 * @brief Return estimated inertia J.
 *
 * @param est        Pointer to estimator object
 * @return Estimated J [kg m^2], or negative value if invalid
 */
float InertiaEstimator_GetJ(const InertiaEstimator_t *est);
/**
 * @brief Return estimated viscous friction B.
 *
 * @param est        Pointer to estimator object
 * @return Estimated B [Nms/rad], or negative value if invalid
 */
float InertiaEstimator_GetB(const InertiaEstimator_t *est);

/**
 * @brief Return whether the current estimate is valid.
 *
 * @param est        Pointer to estimator object
 * @return true if valid estimate available
 */
bool InertiaEstimator_IsValid(const InertiaEstimator_t *est);


void set_inertia_estimation_flag(void);
void motor_inertia_estimation_time_management(void);
void motor_inertia_estimation_init(void);


#endif /* INC_MOTOR_INERTIA_EST_H_ */
