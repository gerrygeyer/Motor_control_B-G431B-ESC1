#include "motor_events.h"
#include <stdbool.h>
/* Faults */
volatile bool oc_trip = false;
volatile bool ot_trip = false;
volatile bool ov_trip = false;

/* User Inputs */
volatile bool btn_stop_edge         = false; 
volatile bool btn_closed_edge       = false;
volatile bool btn_open_edge         = false;
volatile bool btn_gotostart_edge    = false;
volatile bool btn_parameter_id_edge = false;