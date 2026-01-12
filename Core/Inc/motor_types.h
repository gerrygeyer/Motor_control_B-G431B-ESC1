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

    volatile position_information_t position_info;  // Quelle
    motor_state_est_t est;                          // Werte (nicht unbedingt volatile, s.u.)

    volatile bool gotostart_finish_flag;
    volatile bool stop_request_flag;
    volatile bool start_request_flag;
} Motor;
    


extern Motor g_motor;


#endif /*INC_MOTOR_TYPES_H_*/