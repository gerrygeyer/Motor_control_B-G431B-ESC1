#include "motor_hfi.h"
#include "foc_math.h"
#include "parameter.h"


void get_angle_from_frequency(const uint16_t freq, uint16_t *angle_increment) {
    uint16_t x = (uint16_t)(((uint32_t)freq << 16) / (uint32_t)FOC_FREQUENCY); // Q16 format
    *angle_increment += x;
}


int16_t slow_iir_filter(int16_t input, iir_filter_f *filter){
    float y = (float)input * filter->a + filter->y_last * filter->one_min_a;
    filter->y_last = y;
    return (int16_t)CLAMP_INT32_TO_INT16((int32_t)y);
}