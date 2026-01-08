#include "motor_events.h"
#include <stdbool.h>
/* Faults */
volatile bool oc_trip = false;

/* User Inputs */
volatile bool btn_stop_edge      = false;
volatile bool btn_closed_edge    = false;
volatile bool btn_open_edge      = false;
volatile bool btn_gotostart_edge = false;