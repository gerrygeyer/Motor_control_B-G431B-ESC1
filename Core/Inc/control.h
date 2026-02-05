/*
 * control.h
 *
 *  Created on: Nov 22, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

//#define RESET	0
//#define SET		1

void init_control_functions(Control_Loops *ctrl);
void clear_control_parameter(Control_Loops *ctrl);

void reset_pi_integrator(Control_Loops *ctrl);

void update_PI_parameter(Control_Loops *ctrl);

/**
 * @brief       Q15 PI current controller for d- and q-axis.
 *
 * @details     Implements a discrete PI controller using fixed-point arithmetic (Q-format):
 *              - Proportional term in Q14 for higher resolution.
 *              - Integrator term accumulated in Q20 for improved accuracy.
 *              - Anti-windup mechanism via subtraction of the output limitation error.
 *              - Circular voltage limitation using `circle_limitation_Q15()`.
 *
 * @param       error           Control error in Q15 format for d- and q-axis (normalized to I_max).
 * @param       pHandle_foc     Pointer to the FOC control structure (currently unused).
 *
 * @return      Output voltage vector in Q15 format, normalized to V_max and limited in magnitude.
 *
 * @note
 * - Gains `Kp_Q14` and `Ki_Q15` must be precomputed and scaled correctly.
 * - Integrator state is stored in global variable `I_current_buffer_q20` (Q20).
 * - Anti-windup uses global variable `winduptQ15` (Q15).
 * - `circle_limitation_Q15()` ensures that the resulting voltage vector stays within valid limits.
 *
 * @see         circle_limitation_Q15(), CLAMP_INT32_TO_INT16(), Q15 fixed-point arithmetic
 */
dq_t PI_id_iq_Q15(dq_t error, Control_Loops *ctrl, FOC_HandleTypeDef *pHandle_foc);


int16_t pi_speed_q15 (int32_t speed_rpm, int32_t speed_rpm_ref, Control_Loops *ctrl, FOC_HandleTypeDef *pHandle_foc);

#endif /* INC_CONTROL_H_ */
