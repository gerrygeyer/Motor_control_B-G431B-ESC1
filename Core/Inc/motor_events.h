#ifndef MOTOR_EVENTS_H
#define MOTOR_EVENTS_H

#include <stdbool.h>

/* Faults */
extern volatile bool oc_trip;
extern volatile bool ot_trip;
extern volatile bool ov_trip;  

/* User Inputs */
extern volatile bool btn_stop_edge;         // STOP
extern volatile bool btn_closed_edge;       // CLOSED LOOP
extern volatile bool btn_open_edge;         // OPEN LOOP
extern volatile bool btn_gotostart_edge;    // GOTO START

#endif