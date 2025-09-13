/*
 * lowpass_filters.h
 *
 *  Created on: Sep 4, 2025
 *      Author: alper
 */

#ifndef INC_LOWPASS_FILTERS_H_
#define INC_LOWPASS_FILTERS_H_

#include <stdint.h>

typedef struct
{
	volatile float b[2];
	volatile float a;
	volatile float x_prev;
	volatile float y_prev;
}lowpass_filter;

typedef struct
{
	volatile float b[3];
	volatile float a[3];
	volatile float x_prev[2];
	volatile float y_prev[2];
}butter2_filter;

typedef struct
{
	lowpass_filter lp1;
	butter2_filter lp2;
}butter3_filter;

typedef struct
{
	butter2_filter lp1;
	butter2_filter lp2;
}butter4_filter;

typedef struct
{
	lowpass_filter lp1;
	butter4_filter lp2;
}butter5_filter;

typedef struct
{
	butter2_filter lp1;
	butter4_filter lp2;
}butter6_filter;

#define SG_WINDOW 9

typedef struct {
    int32_t buf[SG_WINDOW];
    int idx;
} sg_filter;

void lowpass_filter_init(lowpass_filter *filter, float fc, float fs);
float lowpass_filter_update(lowpass_filter *f, float x);

void butter2_filter_init(butter2_filter *f, float fc, float fs);
float butter2_filter_update(butter2_filter *f, float x);

void butter3_filter_init(butter3_filter *f, float fc, float fs);
float butter3_filter_update(butter3_filter *f, float x);

void butter4_filter_init(butter4_filter *f, float fc, float fs);
float butter4_filter_update(butter4_filter *f, float x);

void butter5_filter_init(butter5_filter *f, float fc, float fs);
float butter5_filter_update(butter5_filter *f, float x);

void butter6_filter_init(butter6_filter *f, float fc, float fs);
float butter6_filter_update(butter6_filter *f, float x);

void sg_filter_init(sg_filter *f);
float sg_filter_update(sg_filter *f, int32_t x, float Ts);

#endif /* INC_LOWPASS_FILTERS_H_ */
