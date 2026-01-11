#include "motor_types.h"
#include <stdbool.h>

Motor g_motor = {
    .state = ST_STOP,
    .speed_ref = 0,
    .position_info = POS_ENCODER,
    .gotostart_finish_flag = false,
    .stop_request_flag = false,
    .start_request_flag = false
};
