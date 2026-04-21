#include "motor_types.h"
#include "parameter.h"
#include <stdbool.h>

Motor g_motor = {
    .state = ST_STOP,
    .speed_ref = 500,
    .position_info = POS_ENCODER,
    .gotostart_finish_flag = false,
    .stop_request_flag = false,
    .start_request_flag = false,
    .recive_command_flag = false
};

Control_Loops ctrl = {
    .motor_params = {
        .Rs = PMSM_RS_OHM,
        .Ls = PMSM_LS,
        .Ke = PMSM_KE,
        .max_speed = MAX_SPEED,
        .max_voltage = MAX_VOLTAGE
    },
    .current = {
        .Kp = {0,Qerror},
        .Ki = {0,Qerror},
        .ff_c1 = {0,Qerror},
        .ff_c2 = {0,Qerror},
        .I_buffer = {0,0},
        .windup = {0,0}
    },
    .speed = {
        .Kp = {0,Qerror},
        .Ki = {0,Qerror},
        .I_buffer = {0,0},
        .windup = {0,0}
    },
    .alpha = 0,
    .new_user_parameter_flag = false,
    .new_parameter_flag = false,
    .nonvalid_values_flag = true
};

FOC_HandleTypeDef foc_values = {

    .speed_ref = 0,
    .speed_q15 = 0,
    .speed = 0,
    .acc_q15 = 0,
    .speed_calc_param = {
        .count_to_angle = 0,
        .uint2rad_q15 = 0
    },
    .theta = 0,
    .theta_openloop = 0,
    .elec_theta_q15 = 0,
    .source_voltage = 0,
    .temperature = 0,

    .I_ref_q15 = {0,0},
    .I_ab_q15 = {0,0},
    .I_alph_bet_q15 = {0,0},
    .I_dq_q15 = {0,0},
    .V_dq_q15 = {0,0},
    .V_ff_q15 = {0,0},
    .V_alph_bet_q15 = {0,0},
    .V_abc_q15 = {0,0,0},
    .Iq_meas_q15 = 0,
    .current_ff_flag = false
};

freq_flag_t freq_flag = {
    .mid = false,
    .low = false,
    .oneHz = false
};