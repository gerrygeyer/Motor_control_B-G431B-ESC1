#ifndef MOTOR_EVENTS_H
#define MOTOR_EVENTS_H

#include <stdbool.h>

/* Faults */
extern volatile bool oc_trip;
extern volatile bool ot_trip;
extern volatile bool ov_trip;  

/* User Inputs */
extern volatile bool btn_stop_edge;
extern volatile bool btn_closed_edge;
extern volatile bool btn_open_edge;
extern volatile bool btn_gotostart_edge;

#endif