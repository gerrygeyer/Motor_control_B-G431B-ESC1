/*
 * math.c
 *
 *  Created on: Jul 2, 2024
 *      Author: Gerry Geyer
 */
#include <math.h>
#include <foc_math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "parameter.h"

/*
 * Sin function:
 * 		Input: theta in int16_t
 * 		Output: sin in int16_t
 */
int16_t sin_t(int16_t y){
	int16_t Output;

	int32_t x_i = ((int32_t)(y * 2048)/ INT16_MAX_VALUE);

	if(x_i < 0){
		x_i = x_i * (-1);
		if(x_i > 1024){
			Output = -sineLookupTable90[(2048 - x_i)];
		}else{
			Output = -sineLookupTable90[x_i];
		}
	}else{
		if(x_i > 1024){
			Output = sineLookupTable90[(2048 - x_i)];
		}else{
			Output = sineLookupTable90[x_i];
		}

	}

	return Output;
}

int16_t cos_t(int16_t y){
	return sin_t(y + INT16_HALF_VALUE);
}




/*
 * create a int16_t value from float. to calculate, use Bitshift >>= 15 after calculation.
 */
int16_t get_INT16_representation(float value){
	return (value * INT16_MAX_VALUE);
}


uint32_t sqrt_fast_uint(uint32_t n) {
    if (n == 0) return 0;

    uint8_t b = 31 - __builtin_clz(n);
    uint32_t x = 1 << (b >> 1); // Initial guess: 2^(b/2)

    x = (x + n / x) >> 1;
    x = (x + n / x) >> 1;
    x = (x + n / x) >> 1;

    if ((uint64_t)x * x > n) x--;

    return x;
}


dq_t circle_limitation_Q15(dq_t Vdq, const uint32_t max_output){
	dq_t Output;

	int32_t x = Vdq.d;
	int32_t y = Vdq.q;

	uint32_t mag = (uint32_t)(x*x) + (uint32_t)(y*y);

	mag = sqrt_fast_uint(mag);

	if(mag == 0){
		Output.d = 0;
		Output.q = 0;
		return (Output);
	}

	if (mag <= max_output) {
		Output.d = Vdq.d;
		Output.q = Vdq.q;
		return Output;
	}

    // Skalenfaktor vorbereiten in Q15: scale = 32767 / |v|
    // Wir rechnen: scale = (32767 << 15) / mag
    uint32_t scale_q15 = (max_output << 15) / mag;
    // norm[i] = (accel[i] * scale_q15) >> 15
    Output.d = CLAMP_INT32_TO_INT16((int32_t)(((int64_t)x * scale_q15) >> 15));
    Output.q = CLAMP_INT32_TO_INT16((int32_t)(((int64_t)y * scale_q15) >> 15));

    return (Output);
}


fixed16_t get_q_format(float value){
	
	float abs_value = fabs(value);
	fixed16_t Output;
	if(abs_value == 0.0f){
		Output.q = Qerror; 
		Output.value = 0; 
		return Output;
	}
	if(abs_value >= 1.0f){
		for(uint8_t i = 0; i <= 15; i++){
			float scaled_value = abs_value / (1 << i);
			if(scaled_value < 1.0f){
				// do nothing
			}else{
				uint8_t q_format = 15 - i;
				Output.q 		= (q_format_t)(q_format);
				Output.value = (int16_t)CLAMP_INT32_TO_INT16(value * ((1 << (q_format))-1));
				return Output;
			}
		}
	}else{
		for(uint8_t i = 1; i <= 15; i++){
			float scaled_value = abs_value * (1 << i);
			// if((abs_value * (1 << i)) < 1.0f){
			if(scaled_value < 1.0f){
				// do nothing
                if(i == 15){
                    Output.q = (q_format_t)(15); 
                    Output.value = (int16_t)CLAMP_INT32_TO_INT16(value * ((1 << 15)-1));
                    return Output;
                }
			}else{
				uint8_t q_format = 15 + i - 1;
				Output.q 		= (q_format_t)(q_format);
				Output.value 	= (int16_t)CLAMP_INT32_TO_INT16(value * ((1 << (q_format))-1));
				return Output;
			}
		}
	}
	Output.q = Qerror;
	Output.value = 0;
	return Output;
}






/* extern LUT (256 entries, uint16_t, range [0..8192]) */
// extern const uint16_t atan_lut[256];

/**
 * @brief Approximate atan2(beta, alpha) using a 256-entry LUT with linear interpolation.
 *
 * Input:
 *   e->alpha = x
 *   e->beta  = y
 *
 * Output:
 *   uint16_t angle in [0, 65535], representing [0, 2*pi)
 *
 * LUT covers atan(r) for r in [0,1].
 */
uint16_t fast_atan2_u16(const alphabeta32_t *e)
{
    int32_t x;
    int32_t y;
    uint32_t ax;
    uint32_t ay;
    uint32_t ratio_q16;
    uint16_t idx;
    uint16_t frac;
    uint16_t phi_base;
    uint16_t phi_next;
    uint16_t phi_small;
    uint16_t phi;
    uint32_t interp;

    if (e == 0) {
        return 0u;
    }

    x = (int32_t)e->alpha;
    y = (int32_t)e->beta;

    ax = (x < 0) ? (uint32_t)(-x) : (uint32_t)x;
    ay = (y < 0) ? (uint32_t)(-y) : (uint32_t)y;

    /* Zero vector */
    if ((ax == 0u) && (ay == 0u)) {
        return 0u;
    }

    /* Axis cases */
    if (ax == 0u) {
        return (y > 0) ? 16384u : 49152u;   /* pi/2 or 3pi/2 */
    }

    if (ay == 0u) {
        return (x >= 0) ? 0u : 32768u;      /* 0 or pi */
    }

    /*
     * Reduce to first octant:
     * r = min(ax, ay) / max(ax, ay),  r in [0,1]
     *
     * ratio_q16 in [0,65535]
     */
    if (ax >= ay) {
        ratio_q16 = (ay << 16) / ax;
    } else {
        ratio_q16 = (ax << 16) / ay;
    }

    /*
     * Convert Q16 ratio to LUT index + interpolation fraction:
     * idx  = 0..255
     * frac = 0..255
     */
    idx  = (uint16_t)(ratio_q16 >> 8);
    frac = (uint16_t)(ratio_q16 & 0xFFu);

    /*
     * Clamp idx for safe access to idx+1
     */
    if (idx >= 255u) {
        phi_small = atan_lut[255];
    } else {
        phi_base = atan_lut[idx];
        phi_next = atan_lut[idx + 1u];

        /*
         * Linear interpolation:
         * phi_small = phi_base + (phi_next - phi_base) * frac / 256
         */
        interp = (uint32_t)(phi_next - phi_base) * (uint32_t)frac;
        phi_small = (uint16_t)(phi_base + (uint16_t)(interp >> 8));
    }

    /*
     * Expand from [0, pi/4] to [0, pi/2]
     */
    if (ax >= ay) {
        phi = phi_small;
    } else {
        phi = (uint16_t)(16384u - phi_small);   /* pi/2 - phi_small */
    }

    /*
     * Quadrant mapping
     */
    if (x >= 0) {
        if (y >= 0) {
            /* Q1 */
            return phi;
        } else {
            /* Q4 */
            return (uint16_t)(65536u - phi);
        }
    } else {
        if (y >= 0) {
            /* Q2 */
            return (uint16_t)(32768u - phi);
        } else {
            /* Q3 */
            return (uint16_t)(32768u + phi);
        }
    }
}