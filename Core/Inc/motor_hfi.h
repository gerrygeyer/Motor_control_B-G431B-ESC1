#ifndef MOTOR_HFI_H
#define MOTOR_HFI_H
#include "parameter.h"
#include <stdint.h>

void get_angle_from_frequency(const uint16_t freq, uint16_t *angle_increment);
int16_t slow_iir_filter(int16_t input, iir_filter_f *filter);


#endif // MOTOR_HFI_H