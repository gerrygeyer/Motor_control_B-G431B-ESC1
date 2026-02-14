/*
motor_types.h
*/



#ifndef INC_MOTOR_TYPES_H_
#define INC_MOTOR_TYPES_H_


#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ST_STOP = 0,
    ST_CLOSEDLOOP,
    ST_OPENLOOP,
    ST_GOTOSTART,
    ST_PARAMETER_ID,
    ST_FAULT,
    ST_MAX
} MotorState;

typedef enum {
    FAST_OFF = 0,        // PWM aus / keine Regelung
    FAST_SPEED_CTRL,     // FOC + Speed loop
    FAST_OPENLOOP,       // OpenLoop
    FAST_GOTOSTART,      // Startsequenz
    FAST_SENSORLESS      // placeholder
} motor_fast_mode_t;

typedef enum {
    PIDM_IDLE = 0,
    PIDM_START,
    PIDM_EST_R,
    PIDM_EST_L,
    PIDM_EST_J,
    PIDM_DONE,
    PIDM_ERROR
} motor_pidm_state_t;

typedef struct {
    float Rs_ohm;
    float Ls_h;
    float J_kgm2;
    bool  valid;
} motor_param_result_t;


typedef enum {
    POS_ENCODER = 0,
    POS_SENSORLESS
} position_information_t;

typedef struct {
    int32_t  theta_e;      // elektrische Position (z.B. Q31, [0..2pi))
    int32_t  omega_e;      // elektrische Winkelgeschwindigkeit (z.B. Q31 oder int32 rpm*scale)
    bool     valid;        // z.B. Observer noch nicht eingerastet
} motor_state_est_t;

typedef struct {
    volatile MotorState state;
    volatile int16_t speed_ref;

    volatile position_information_t position_info;  
    motor_state_est_t est;                          

    volatile motor_fast_mode_t fast_mode;          // Fast-loop dispatch (20 kHz)

    /* Parameter-ID (nested SM) control + results */
    volatile motor_pidm_state_t pidm_state;
    volatile bool pidm_start_request_flag;         // set by Service/command layer
    volatile bool pidm_abort_request_flag;         // optional abort
    motor_param_result_t pidm_result;


    volatile bool gotostart_finish_flag;
    volatile bool stop_request_flag;
    volatile bool start_request_flag;

    volatile bool recive_command_flag; // set by UART callback, cleared by Service layer, optional for safety monitoring
} Motor;
    


extern Motor g_motor;


#endif /*INC_MOTOR_TYPES_H_*/