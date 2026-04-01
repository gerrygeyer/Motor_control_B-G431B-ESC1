#include "motor_inertia_est.h"
#include "parameter.h"
#include "motor_task.h"
#include <math.h>


InertiaEstimator_t inertia_est;
static RingBuf7n16_t iq_buffer;
static RingBuf7n16_t speed_buffer;
static fixed16_t ds_dt_q15;

static inline void RB_Init(RingBuf7n16_t *rb);
static inline void RB_Push(RingBuf7n16_t *rb, int16_t x);
static inline int32_t SG7_Diff_speed_q15(const RingBuf7n16_t *rb, fixed16_t g_q15);
static inline int32_t RB_GetK(const RingBuf7n16_t *rb, uint8_t k);

void set_inertia_estimation_flag(void){
    inertia_est.start_flag = true;
}

void motor_inertia_estimation_time_management(void){
    if(!inertia_est.start_flag){
        return;
    }
    inertia_est.start_flag = false; 

    float iq_A;
    float omega_rad_s;
    float alpha_rad_s2;

    RB_Push(&speed_buffer, FOC_GetValues()->speed_q15);
    RB_Push(&iq_buffer, FOC_GetValues()->I_dq_q15.q);


    int16_t acc_q15 = FOC_GetValues()->acc_q15 = SG7_Diff_speed_q15(&speed_buffer, ds_dt_q15);
    int16_t omega_q15 = RB_GetK(&speed_buffer, 3);
    int16_t iq_q15 = RB_GetK(&iq_buffer, 3);

    iq_A = (float)iq_q15 / (float)Q10; // max current = 32 A [Q10]
    omega_rad_s = ((float)omega_q15 / (float)Q15) * MAX_SPEED * RPM_TO_RAD_S; // convert back to rad/s
    alpha_rad_s2 = ((float)acc_q15 / (float)Q15) * MAX_SPEED * RPM_TO_RAD_S; // convert back to rad/s^2

    InertiaEstimator_Update(&inertia_est, iq_A, omega_rad_s, alpha_rad_s2);

    if (InertiaEstimator_IsValid(&inertia_est))
    {
        inertia_est.j_hat = InertiaEstimator_GetJ(&inertia_est);

        /* Debug / Logging */
        // debug.j_hat = j_hat;
        // debug.theta1 = inertia_est.theta[0];
        // debug.theta2 = inertia_est.theta[1];
        // debug.theta3 = inertia_est.theta[2];
        // debug.rls_error = inertia_est.last_error;
    }


}

void motor_inertia_estimation_init(void){
    RB_Init(&speed_buffer);
    RB_Init(&iq_buffer);

	float ds_dt_f = (float)M_FREQUENCY / ((float)MAX_SPEED * RPM_TO_RAD_S); // (1/w_max)*(1/dt)
	ds_dt_q15 = get_q_format(ds_dt_f);
 /* Beispielwerte:
       lambda = 0.995
       p0     = 1000
       kt     = 0.035 Nm/A
       J plausibel zwischen 1e-6 und 1e-2 kgm^2
    */
    InertiaEstimator_Init(&inertia_est,
                          0.995f,
                          1000.0f,
                          0.035f,
                          1e-6f,
                          1e-2f);

    inertia_est.iq_min = 0.2f;      // minimal current for valid estimation
    inertia_est.alpha_min = 5.0f;   // minimal acceleration for valid estimation
    inertia_est.start_flag = false;
}



/* ChatGPT say its better than using CLAMP Macro */
static inline float clampf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

/* Symmetrize the covariance matrix: take the average of the off-diagonal elements */
static void symmetrize_P(InertiaEstimator_t *est)
{
    float p01 = 0.5f * (est->P[0][1] + est->P[1][0]);
    float p02 = 0.5f * (est->P[0][2] + est->P[2][0]);
    float p12 = 0.5f * (est->P[1][2] + est->P[2][1]);

    est->P[0][1] = p01;
    est->P[1][0] = p01;

    est->P[0][2] = p02;
    est->P[2][0] = p02;

    est->P[1][2] = p12;
    est->P[2][1] = p12;
}

static void update_derived_parameters(InertiaEstimator_t *est)
{
    const float theta1 = est->theta[0];
    const float theta2 = est->theta[1];
    const float theta3 = est->theta[2];

    est->valid = false;
    est->j_hat = -1.0f;
    est->b_hat = 0.0f;
    est->load_over_j_hat = 0.0f;

    if (fabsf(theta1) < INERTIA_ESTIMATOR_EPS)
    {
        return;
    }

    est->j_hat = est->kt / theta1;

    if ((est->j_hat < est->j_min) || (est->j_hat > est->j_max) || !isfinite(est->j_hat))
    {
        est->j_hat = -1.0f;
        return;
    }

    est->b_hat = -theta2 * est->j_hat;
    est->load_over_j_hat = -theta3;

    if (!isfinite(est->b_hat) || !isfinite(est->load_over_j_hat)) // This should not happen if theta1 is valid, but we check anyway
    {
        est->j_hat = -1.0f;
        est->b_hat = 0.0f;
        est->load_over_j_hat = 0.0f;
        return;
    }

    est->valid = true;
}

void InertiaEstimator_Init(InertiaEstimator_t *est,
                           float lambda,
                           float p0,
                           float kt,
                           float j_min,
                           float j_max)
{
    if (est == NULL)
    {
        return;
    }

    est->theta[0] = 0.0f;
    est->theta[1] = 0.0f;
    est->theta[2] = 0.0f;

    est->P[0][0] = p0;   est->P[0][1] = 0.0f; est->P[0][2] = 0.0f;
    est->P[1][0] = 0.0f; est->P[1][1] = p0;   est->P[1][2] = 0.0f;
    est->P[2][0] = 0.0f; est->P[2][1] = 0.0f; est->P[2][2] = p0;

    est->lambda = clampf(lambda, 0.90f, 1.0f);
    est->kt = kt;

    est->j_hat = -1.0f;
    est->b_hat = 0.0f;
    est->load_over_j_hat = 0.0f;

    est->j_min = j_min;
    est->j_max = j_max;

    est->iq_min = 0.05f;
    est->alpha_min = 1.0f;

    est->last_error = 0.0f;
    est->last_y_hat = 0.0f;
    est->last_den = 0.0f;

    est->update_count = 0u;
    est->initialized = true;
    est->enabled = true;
    est->valid = false;
}

void InertiaEstimator_Reset(InertiaEstimator_t *est, float p0)
{
    if ((est == NULL) || (!est->initialized))
    {
        return;
    }

    est->theta[0] = 0.0f;
    est->theta[1] = 0.0f;
    est->theta[2] = 0.0f;

    est->P[0][0] = p0;   est->P[0][1] = 0.0f; est->P[0][2] = 0.0f;
    est->P[1][0] = 0.0f; est->P[1][1] = p0;   est->P[1][2] = 0.0f;
    est->P[2][0] = 0.0f; est->P[2][1] = 0.0f; est->P[2][2] = p0;

    est->j_hat = -1.0f;
    est->b_hat = 0.0f;
    est->load_over_j_hat = 0.0f;

    est->last_error = 0.0f;
    est->last_y_hat = 0.0f;
    est->last_den = 0.0f;

    est->update_count = 0u;
    est->valid = false;
}

void InertiaEstimator_Enable(InertiaEstimator_t *est, bool enable)
{
    if (est == NULL)
    {
        return;
    }

    est->enabled = enable;
}

void InertiaEstimator_Update(InertiaEstimator_t *est,
                             float iq,
                             float omega,
                             float alpha)
{
    float phi[3];
    float v[3];
    float K[3];
    float P_old[3][3];
    float y_hat;
    float error;
    float den;
    uint32_t i;
    uint32_t j;

    if ((est == NULL) || (!est->initialized) || (!est->enabled))
    {
        return;
    }

    if ((!isfinite(iq)) || (!isfinite(omega)) || (!isfinite(alpha)))
    {
        return;
    }

    if (fabsf(iq) < est->iq_min)
    {
        return;
    }

    if (fabsf(alpha) < est->alpha_min)
    {
        return;
    }

    phi[0] = iq;
    phi[1] = omega;
    phi[2] = 1.0f;

    y_hat = phi[0] * est->theta[0]
          + phi[1] * est->theta[1]
          + phi[2] * est->theta[2];

    error = alpha - y_hat;

    v[0] = est->P[0][0] * phi[0] + est->P[0][1] * phi[1] + est->P[0][2] * phi[2];
    v[1] = est->P[1][0] * phi[0] + est->P[1][1] * phi[1] + est->P[1][2] * phi[2];
    v[2] = est->P[2][0] * phi[0] + est->P[2][1] * phi[1] + est->P[2][2] * phi[2];

    den = est->lambda + phi[0] * v[0] + phi[1] * v[1] + phi[2] * v[2];

    est->last_y_hat = y_hat;
    est->last_error = error;
    est->last_den = den;

    if ((!isfinite(den)) || (fabsf(den) < INERTIA_ESTIMATOR_EPS))
    {
        return;
    }

    K[0] = v[0] / den;
    K[1] = v[1] / den;
    K[2] = v[2] / den;

    est->theta[0] += K[0] * error;
    est->theta[1] += K[1] * error;
    est->theta[2] += K[2] * error;

    for (i = 0u; i < 3u; i++)
    {
        for (j = 0u; j < 3u; j++)
        {
            P_old[i][j] = est->P[i][j];
        }
    }

    for (i = 0u; i < 3u; i++)
    {
        for (j = 0u; j < 3u; j++)
        {
            est->P[i][j] = (P_old[i][j] - K[i] * v[j]) / est->lambda;
        }
    }

    symmetrize_P(est);
    update_derived_parameters(est);

    est->update_count++;
}

float InertiaEstimator_GetJ(const InertiaEstimator_t *est)
{
    if ((est == NULL) || (!est->valid))
    {
        return -1.0f;
    }

    return est->j_hat;
}

bool InertiaEstimator_IsValid(const InertiaEstimator_t *est)
{
    if (est == NULL)
    {
        return false;
    }

    return est->valid;
}



// ############ Savitzky-Golan Filter (n = 7, p = 2) ############

static inline void RB_Init(RingBuf7n16_t *rb)
{
    for (int i = 0; i < 7; i++) rb->buf[i] = 0;
    rb->head = 0;
    rb->filled = 0;
}

static inline void RB_Push(RingBuf7n16_t *rb, int16_t x)
{
    rb->buf[rb->head] = x;

    rb->head++;
    if (rb->head >= 7) rb->head = 0;

    if (rb->filled < 7) rb->filled++;
}


// k=0 -> newest, k=1 -> previous, ...
static inline int32_t RB_GetK(const RingBuf7n16_t *rb, uint8_t k)
{
    // assumes k < rb->filled
    int32_t idx = (int32_t)rb->head - 1 - (int32_t)k;
    while (idx < 0) idx += 7;      // wrap negative
    // idx is now 0..6
    return rb->buf[(uint8_t)idx];
}

// k=0 newest, ... k=6 oldest
static inline int32_t SG7_Diff_speed_q15(const RingBuf7n16_t *rb, fixed16_t g_q15)
{
    if (rb->filled < 7) return 0;

    // Zuordnung für Zentrum bei k-3:
    // x(+3)=k   -> GetK(0)
    // x(+2)=k-1 -> GetK(1)
    // x(+1)=k-2 -> GetK(2)
    // x(-1)=k-4 -> GetK(4)
    // x(-2)=k-5 -> GetK(5)
    // x(-3)=k-6 -> GetK(6)

    int32_t xp3 = RB_GetK(rb, 0);
    int32_t xp2 = RB_GetK(rb, 1);
    int32_t xp1 = RB_GetK(rb, 2);
    int32_t xm1 = RB_GetK(rb, 4);
    int32_t xm2 = RB_GetK(rb, 5);
    int32_t xm3 = RB_GetK(rb, 6);

    int32_t d3 = xp3 - xm3; // (x+3 - x-3)
    int32_t d2 = xp2 - xm2; // (x+2 - x-2)
    int32_t d1 = xp1 - xm1; // (x+1 - x-1)

    // S = 3*d3 + 2*d2 + 1*d1  (noch ohne /28 und ohne fs)
    // Multiplikationen durch 2/3 als Shift+Add:
    int32_t S = 0;
    S += ((int32_t)d3 << 1) + (int32_t)d3; // 3*d3
    S += ((int32_t)d2 << 1);              // 2*d2
    S += (int32_t)d1;                     // 1*d1

    // ds/dt in Q15: (S_q15 * g_q15)>>15
    int32_t tmp = S * (int32_t)g_q15.value;     // Q15*Q15 = Q30
    int32_t dsdt_q15 = (int32_t)(tmp >> g_q15.q);

    return dsdt_q15; // normierte Ableitung (1/s), gehört zeitlich zu k-3
}