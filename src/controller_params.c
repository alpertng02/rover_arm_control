/*
 * controllers.c
 *
 *  Created on: Sep 4, 2025
 *      Author: alper
 */

#include "controller_params.h"
#include <math.h>

#define CLAMP(x, a, b) (((x) < (a)) ? (a) : \
			((b) < (x)) ? (b) : (x))

void pid_params_init(pid_params* pid,
		float kp, float ki, float kd,
		float p_bound, float i_bound, float d_bound, float fs)
{

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->prev_err = 0;
	pid->i = 0;
	pid->p_bound = p_bound;
	pid->i_bound = i_bound;
	pid->d_bound = d_bound;

	pid->delta_t = 1.0f / fs;

}

float pid_params_update(pid_params* pid, float err)
{
	float p = pid->kp * err;
	float i = (pid->ki * err) * pid->delta_t;
	float d = (pid->kd)*(err-pid->prev_err) / pid->delta_t;

	pid->i += i;

	p = CLAMP(p, -pid->p_bound, pid->p_bound);
	pid->i = CLAMP(pid->i, -pid->i_bound, pid->i_bound);
	d = CLAMP(d, -pid->d_bound, pid->d_bound);

	pid->prev_err = err;

	float res = (p + pid->i + d);
	return res;
}

float pid_params_update_d_filtered(pid_params* pid, float err, float d_filtered)
{
	float p = pid->kp * err;
	float i = (pid->ki * err) * pid->delta_t;
	float d = (pid->kd) * d_filtered;

	pid->i += i;

	p = CLAMP(p, -pid->p_bound, pid->p_bound);
	pid->i = CLAMP(pid->i, -pid->i_bound, pid->i_bound);
	d = CLAMP(d, -pid->d_bound, pid->d_bound);

	pid->prev_err = err;

	float res = (p + pid->i + d);
	return res;
}

void feedforward_params_init(feedforward_params* ff,
		float kv, float ka, float kj,float v_offset)
{
	ff->kv = kv;
	ff->ka = ka;
	ff->kj = kj;

	ff->v_offset = v_offset;
}

float feedforward_params_update(feedforward_params* ff,
		float vel_ref, float acc_ref, float j_ref)
{
	float v_ff = vel_ref * ff->kv;
	if(fabsf(vel_ref) < 10.0f)
	{
		float dir = vel_ref >= 0.0f ? 1.0f : -1.0f;
		v_ff += dir * ff->v_offset;
	}
	float a_ff = acc_ref * ff->ka;
	float j_ff = j_ref   * ff->kj;

	float ff_term = v_ff + a_ff + j_ff;

	return ff_term;
}

