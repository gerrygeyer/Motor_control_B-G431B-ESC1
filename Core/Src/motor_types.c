#include "motor_types.h"

Motor g_motor = {
    .state = ST_STOP,
    .speed_ref = 0,
    .position_info = POS_ENCODER
};