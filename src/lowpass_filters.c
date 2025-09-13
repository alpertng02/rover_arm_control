/*
 * lowpass_filters.c
 *
 *  Created on: Sep 4, 2025
 *      Author: alper
 */

#include "lowpass_filters.h"
#include <math.h>
#include <stdint.h>

#ifdef M_PI
#define M_PI_F 	((float)(M_PI))
#else
#define M_PI_F 	(3.141592653589793f)
#endif

void lowpass_filter_init(lowpass_filter* filter, float fc, float fs)
{
    float k = tanf(M_PI_F * fc / fs);

    filter->b[0] = k / (1.0f + k);
    filter->b[1] = filter->b[0];
    filter->a = (k - 1.0f) / (1.0f + k);

    // reset state
    filter->x_prev = 0.0f;
    filter->y_prev = 0.0f;
}

float lowpass_filter_update(lowpass_filter *f, float x)
{
    float y = f->b[0] * x + f->b[1] * f->x_prev - f->a * f->y_prev;

    // update state
    f->x_prev = x;
    f->y_prev = y;

    return y;
}

void butter2_filter_init(butter2_filter *f, float fc, float fs)
{
    float omega = tanf(M_PI_F * fc / fs); // pre-warped frequency
    float omega2 = omega * omega;
    float sqrt2 = 1.41421356237f; // sqrt(2) for Butterworth

    float a0 = 1.0f + sqrt2 * omega + omega2;

    f->b[0] = omega2 / a0;
    f->b[1] = 2.0f * f->b[0];
    f->b[2] = f->b[0];

    f->a[0] = (2.0f * (omega2 - 1.0f)) / a0;
    f->a[1] = (1.0f - sqrt2 * omega + omega2) / a0;

    // reset state
    f->x_prev[0] = f->x_prev[1] = 0.0f;
    f->y_prev[0] = f->y_prev[1] = 0.0f;
}

float butter2_filter_update(butter2_filter *f, float x)
{
    float y = f->b[0] * x + f->b[1] * f->x_prev[0] + f->b[2] * f->x_prev[1]
              - f->a[0] * f->y_prev[0] - f->a[1] * f->y_prev[1];

    // update state
    f->x_prev[1] = f->x_prev[0];
    f->x_prev[0] = x;

    f->y_prev[1] = f->y_prev[0];
    f->y_prev[0] = y;

    return y;
}

void butter3_filter_init(butter3_filter *f, float fc, float fs)
{
	lowpass_filter_init(&(f->lp1), fc, fs);
	butter2_filter_init(&(f->lp2), fc, fs);
}

float butter3_filter_update(butter3_filter *f, float x)
{
	float y1 = lowpass_filter_update(&(f->lp1), x);
	float y2 = butter2_filter_update(&(f->lp2), y1);

	return y2;
}


void butter4_filter_init(butter4_filter *f, float fc, float fs)
{
	butter2_filter_init(&(f->lp1), fc, fs);
	butter2_filter_init(&(f->lp2), fc, fs);
}

float butter4_filter_update(butter4_filter *f, float x)
{
	float y1 = butter2_filter_update(&(f->lp1), x);
	float y2 = butter2_filter_update(&(f->lp2), y1);

	return y2;
}

void butter5_filter_init(butter5_filter *f, float fc, float fs)
{
	lowpass_filter_init(&(f->lp1), fc, fs);
	butter4_filter_init(&(f->lp2), fc, fs);
}

float butter5_filter_update(butter5_filter *f, float x)
{
	float y1 = lowpass_filter_update(&(f->lp1), x);
	float y2 = butter4_filter_update(&(f->lp2), y1);

	return y2;
}


void butter6_filter_init(butter6_filter *f, float fc, float fs)
{
	butter2_filter_init(&(f->lp1), fc, fs);
	butter4_filter_init(&(f->lp2), fc, fs);
}

float butter6_filter_update(butter6_filter *f, float x)
{
	float y1 = butter2_filter_update(&(f->lp1), x);
	float y2 = butter4_filter_update(&(f->lp2), y1);

	return y2;
}


void sg_filter_init(sg_filter *f) {
    f->idx = 0;
    for (int i = 0; i < SG_WINDOW; i++) f->buf[i] = 0;
}

// push new position sample, return estimated velocity
float sg_filter_update(sg_filter *f, int32_t pos, float Ts) {
    f->buf[f->idx] = pos;
    f->idx = (f->idx + 1) % SG_WINDOW;

    // Map indices: oldest = idx, newest = idx-1
    int base = f->idx; // points to oldest

    int i0 = (base + 0) % SG_WINDOW; // n-8
    int i1 = (base + 1) % SG_WINDOW; // n-7
    int i2 = (base + 2) % SG_WINDOW; // n-6
    int i3 = (base + 3) % SG_WINDOW; // n-5
    //int i4 = (base + 4) % SG_WINDOW; // n-4
    int i5 = (base + 5) % SG_WINDOW; // n-3
    int i6 = (base + 6) % SG_WINDOW; // n-2
    int i7 = (base + 7) % SG_WINDOW; // n-1
    int i8 = (base + 8) % SG_WINDOW; // n

    // Apply SG coefficients: [-3 -2 -1 0 +1 +2 +3] / (28*Ts)
    int32_t num = (-4*f->buf[i0] - 3*f->buf[i1] - 2*f->buf[i2] - f->buf[i3]
                   + f->buf[i5] + 2*f->buf[i6] + 3*f->buf[i7] + 4*f->buf[i8]);

    return (float)num / (60.0f * Ts);
}
