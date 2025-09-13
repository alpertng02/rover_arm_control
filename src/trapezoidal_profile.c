/*
 * trapezoidal_profile.c
 *
 *  Created on: Sep 4, 2025
 *      Author: alper
 */


#include "trapezoidal_profile.h"
#include <math.h>

void trapezoidal_profile_set_trajectory(trapezoidal_profile *profile, float target_pos, float initial_pos, float initial_vel, float final_vel, float max_velocity, float max_accel, float max_deaccel, float fs)
{

	const float pos_change = target_pos - initial_pos;

	float vel_max = fabsf(max_velocity);
	float acc = fabsf(max_accel);
	float dec = fabsf(max_deaccel);

	if(pos_change == 0)
	{
		profile->accel_samples = 0;
		profile->noaccel_samples = 0;
		profile->deaccel_samples = 0;

		profile->current_samples = 0;

		profile->target_position = target_pos;
		return;
	}

	float dir = (pos_change >= 0) ? 1.0f : -1.0f;
	float pos_change_mag = fabsf(pos_change);

	float v0p = dir * initial_vel;
	float vfp = dir * final_vel;

	if(vel_max < fabsf(v0p))
	{
		vel_max = fabsf(v0p);
	}

	float pos_change_accel = fmaxf(0.0f, (vel_max*vel_max - v0p*v0p) / (2.0f*acc));;
	float pos_change_deaccel = (vel_max*vel_max - vfp*vfp)/(2.0f*dec);
	float pos_change_noaccel = 0.0f;

	float t_accel = 0.0f;
	float t_deaccel = 0.0f;
	float t_noaccel = 0.0f;
	if(pos_change_mag < pos_change_accel + pos_change_deaccel)
	{
		vel_max = sqrtf(fmaxf(0.0f, (2.0f*pos_change_mag*acc*dec + dec*v0p*v0p + acc*vfp*vfp) / (acc + dec)));

		t_accel = (vel_max - v0p) / acc;
		t_deaccel = (vel_max - vfp) / dec;
		t_noaccel = 0;
		if(t_accel < 0.0f )
		{
			t_accel = 0.0f;
			t_deaccel = fabsf(v0p) / dec;
		}
	}
	else
	{
		t_accel = fabsf(vel_max - v0p) / acc;
		t_deaccel = fabsf(vel_max - vfp) / dec;
		pos_change_noaccel = pos_change_mag - (pos_change_accel + pos_change_deaccel);
		t_noaccel = pos_change_noaccel / vel_max;
	}

	profile->accel_samples = llroundf(t_accel * fs);
	profile->noaccel_samples = llroundf(profile->accel_samples + t_noaccel * fs);
	profile->deaccel_samples = llroundf(profile->noaccel_samples + t_deaccel * fs);

	profile->current_samples = 0;

	profile->target_position = target_pos;
	profile->max_velocity = (dir * vel_max);
	profile->max_accel = (dir * acc);
	profile->max_deaccel = (dir * dec);
	profile->initial_velocity = v0p;
	profile->final_velocity = vfp;
	profile->initial_position = initial_pos;

	float t1 = t_accel;
	float t2 = t1 + t_noaccel;

	profile->t1 = t1;
	profile->t2 = t2;

	profile->noacc_offset = t1 * profile->initial_velocity   +
			(t1*t1 * 0.5f) * profile->max_accel;
	profile->dec_offset = profile->noacc_offset +
					(t2-t1) * profile->max_velocity;

}


void trapezoidal_profile_update(trapezoidal_profile *profile, int64_t n, float fs)
{
	float pos_ref = profile->initial_position;
	float vel_ref = 0.0f;
	float acc_ref = 0.0f;

	const float t = (float)n / fs;
	const float t1 = profile->t1;
	const float t2 = profile->t2;

	const float vel_max = profile->max_velocity;
	const float acc = profile->max_accel;
	const float dec = profile->max_deaccel;
	const float v0 = profile->initial_velocity;
	const float vf = profile->final_velocity;

	if(n > profile->deaccel_samples)
	{
		pos_ref = profile->target_position;
		vel_ref = vf;
		acc_ref = 0.0f;
		n = profile->deaccel_samples + 1;
	}
	else if(n > profile->noaccel_samples)
	{
		float tao = (t-t2);

		pos_ref +=  profile->dec_offset  	+
				tao * vel_max 				-
				(0.5f*tao*tao) * dec;

		vel_ref = vel_max - dec * tao;

		acc_ref = -dec;
	}
	else if (n > profile->accel_samples)
	{
		float tao = (t-t1);

		pos_ref += profile->noacc_offset 	+
				tao * vel_max;

		vel_ref = vel_max;

		acc_ref = 0;
	}
	else
	{
		pos_ref += t * profile->initial_velocity +
				(0.5f*t*t) * acc;

		vel_ref = v0 + acc * t;

		acc_ref = acc;
	}

	profile->pos_ref = pos_ref;
	profile->vel_ref = vel_ref;
	profile->acc_ref = acc_ref;

	profile->current_samples = n;
}

float trapezoidal_profile_get_position_ref(trapezoidal_profile *profile, int64_t n, float fs)
{
	float pos_ref = profile->initial_position;

	const float t = (float)n / fs;
	const float t1 = profile->t1;
	const float t2 = profile->t2;

	const float vel_max = profile->max_velocity;
	const float acc = profile->max_accel;
	const float dec = profile->max_deaccel;
	const float v0 = profile->initial_velocity;
	const float vf = profile->final_velocity;

	if(n > profile->deaccel_samples)
	{
		pos_ref = profile->target_position;
	}
	else if(n > profile->noaccel_samples)
	{
		float tao = (t-t2);

		pos_ref +=  profile->dec_offset  	+
				tao * vel_max 				-
				(0.5f*tao*tao) * dec;
	}
	else if (n > profile->accel_samples)
	{
		float tao = (t-t1);

		pos_ref += profile->noacc_offset 	+
				tao * vel_max;
	}
	else
	{
		pos_ref += t * v0 +
				(0.5f*t*t) * acc;
	}

	return pos_ref;
}

float trapezoidal_profile_get_velocity_ref(trapezoidal_profile *profile, int64_t n, float fs)
{
	float vel_ref = 0.0f;

	const float t = (float)n / fs;
	const float t1 = profile->t1;
	const float t2 = profile->t2;

	const float vel_max = profile->max_velocity;
	const float acc = profile->max_accel;
	const float dec = profile->max_deaccel;
	const float v0 = profile->initial_velocity;
	const float vf = profile->final_velocity;


	if(n > profile->deaccel_samples)
	{
		vel_ref = 0.0f;
	}
	else if(n > profile->noaccel_samples)
	{
		float tao = (t-t2);

		vel_ref = vel_max - dec * tao;
	}
	else if (n > profile->accel_samples)
	{
		vel_ref = vel_max;
	}
	else
	{
		vel_ref = v0 + acc * t;
	}

	return vel_ref;
}

float trapezoidal_profile_get_acceleration_ref(trapezoidal_profile *profile, int64_t n, float fs)
{
	float acc_ref = 0.0f;

	const float acc = profile->max_accel;
	const float dec = profile->max_deaccel;

	if(n > profile->deaccel_samples)
	{
		acc_ref = 0.0f;
	}
	else if(n > profile->noaccel_samples)
	{
		acc_ref = -dec;
	}
	else if (n > profile->accel_samples)
	{
		acc_ref = 0;
	}
	else
	{
		acc_ref = acc;
	}

	return acc_ref;
}
