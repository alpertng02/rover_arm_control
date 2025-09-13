/*
 * sinusoidal_profile.c
 *
 *  Created on: Sep 4, 2025
 *      Author: alper
 */

#include "sinusoidal_profile.h"
#include <math.h>

#ifdef M_PI
#define M_PI_F 	((float)(M_PI))
#else
#define M_PI_F 	(3.141592653589793f)
#endif

#ifdef STM32G431xx
#include "stm32g4xx_ll_cordic.h"
#endif

#ifdef CORDIC

static inline void init_cordic(void)
{
	LL_CORDIC_Config(CORDIC,
	LL_CORDIC_FUNCTION_COSINE, /* cosine function */
	LL_CORDIC_PRECISION_6CYCLES, /* max precision for q1.31 cosine */
	LL_CORDIC_SCALE_0, /* no scale */
	LL_CORDIC_NBWRITE_1, /* One input data: angle. Second input data (modulus) is 1 af
	ter cordic reset */
	LL_CORDIC_NBREAD_2, /* Two output data: cosine, then sine */
	LL_CORDIC_INSIZE_32BITS, /* q1.31 format for input data */
	LL_CORDIC_OUTSIZE_32BITS); /* q1.31 format for output data */
}

// Convert float (-1.0 <= x < 1.0) to Q1.31 fixed-point
static inline int32_t float_to_q31(float theta)
{
	theta = fmodf(theta, 2.0f * M_PI_F);
	if (theta >= M_PI_F)  theta -= 2.0f * M_PI_F;
	if (theta < -M_PI_F) theta += 2.0f * M_PI_F;

	// Convert to Q1.31
	int32_t theta_q31 = (int32_t)(theta * (2147483648.0f / M_PI_F));
	return theta_q31;
}

// Convert Q1.31 fixed-point to float
static inline float q31_to_float(int32_t q)
{
    return (float)q / 2147483648.0f; // divide by 2^31
}

static inline void profile_sin_cos(float theta, float *cos_res, float *sin_res)
{
	// Convert to Q1.31
	int32_t theta_q31 = float_to_q31(theta);
    LL_CORDIC_WriteData(CORDIC, theta_q31);

	int32_t cos_output = (int32_t)LL_CORDIC_ReadData(CORDIC);
	int32_t sin_output = (int32_t)LL_CORDIC_ReadData(CORDIC);

	*cos_res = q31_to_float(cos_output);
	*sin_res = q31_to_float(sin_output);
}

static inline float profile_cos(float theta)
{
	float cos_res, sin_res;
	profile_sin_cos(theta, &cos_res, &sin_res);
	return cos_res;
}

static inline float profile_sin(float theta)
{
	float cos_res, sin_res;
	profile_sin_cos(theta, &cos_res, &sin_res);
	return sin_res;
}

#else

static inline float profile_cos(float theta)
{
	return cosf(theta);
}

static inline float profile_sin(float theta)
{
	return sinf(theta);
}

static inline void profile_sin_cos(float theta, float *cos_res, float *sin_res)
{
	*cos_res = cosf(theta);
	*sin_res = sinf(theta);
}

#endif

void sinusoidal_profile_set_trajectory(sinusoidal_profile *profile, float target_pos, float initial_pos, float max_velocity, float jerk_acc, float jerk_dec, float fs)
{

#ifdef CORDIC
	init_cordic();
#endif

	profile->initial_position = initial_pos;
	profile->target_position = target_pos;

	const float pos_change = target_pos - initial_pos;

	if(pos_change == 0)
	{
		profile->accel_samples = 0;
		profile->noaccel_samples = 0;
		profile->deaccel_samples = 0;

		profile->current_samples = 0;
		return;
	}

	float dir = (pos_change >= 0) ? 1.0f : -1.0f;
	float p_abs = fabsf(pos_change);
	float vel_max = fabsf(max_velocity);
	float j_acc = fabsf(jerk_acc);
	float j_dec = fabsf(jerk_dec);

	float T_acc = sqrtf((2.0f*M_PI_F*vel_max) / j_acc);
	float T_dec = sqrtf((2.0f*M_PI_F*vel_max) / j_dec);
	float T_noacc = 0.0f;

	float p_acc = (j_acc*T_acc*T_acc*T_acc)/(4.0f*M_PI_F);
	float p_dec = (j_dec*T_dec*T_dec*T_dec)/(4.0f*M_PI_F);
	float p_noacc = 0.0f;

	if(p_abs < p_acc + p_dec)
	{
		float scale = p_abs / (p_acc + p_dec);
	    T_acc = T_acc * cbrt(scale);
	    T_dec = T_dec * cbrt(scale);

	    p_acc = (j_acc * T_acc*T_acc*T_acc) / (4.0f*M_PI_F);
	    p_dec = (j_dec * T_dec*T_dec*T_dec) / (4.0*M_PI_F);

	    T_noacc = 0.0f;
	}
	else
	{
	    p_noacc = p_abs - (p_acc + p_dec);
	    T_noacc = p_noacc / vel_max;
	}

    vel_max = j_acc * T_acc*T_acc / (2.0f*M_PI_F);
    p_acc = (j_acc*T_acc*T_acc*T_acc)/(4.0f*M_PI_F);

	profile->accel_samples = llroundf(T_acc * fs);
	profile->noaccel_samples = llroundf(profile->accel_samples + T_noacc * fs);
	profile->deaccel_samples = llroundf(profile->noaccel_samples + T_dec * fs);

	profile->current_samples = 0;

	profile->target_position = target_pos;
	profile->max_velocity = (dir * vel_max);
	profile->jerk_acc = (dir * j_acc);
	profile->jerk_dec = (dir * j_dec);

	float t1 = T_acc;
	float t2 = t1 + T_noacc;

	profile->T_acc = T_acc;
	profile->T_dec = T_dec;
	profile->t1 = t1;
	profile->t2 = t2;

	profile->noacc_offset = dir * j_acc * T_acc*T_acc*T_acc / (4.0f*M_PI_F);

	profile->dec_offset = profile->noacc_offset+
						(t2-t1) * profile->max_velocity;

	profile->omega_acc = (2.0f * M_PI_F) / T_acc;
	profile->omega_dec = (2.0f * M_PI_F) / T_dec;

	profile->pos_equation_constants[0] = (profile->jerk_acc*T_acc / (4.0f*M_PI_F));
	profile->pos_equation_constants[1] = (profile->jerk_acc*T_acc*T_acc*T_acc) / (8.0f*M_PI_F*M_PI_F*M_PI_F);
	profile->pos_equation_constants[2] = (profile->jerk_dec*T_dec / (4.0f*M_PI_F));
	profile->pos_equation_constants[3] = (profile->jerk_dec*T_dec*T_dec*T_dec) / (8.0f*M_PI_F*M_PI_F*M_PI_F);

	profile->vel_equation_constants[0] = (profile->jerk_acc*T_acc / (2.0f*M_PI_F));
	profile->vel_equation_constants[1] = (profile->jerk_acc*T_acc*T_acc/(4.0f*M_PI_F*M_PI_F));
	profile->vel_equation_constants[2] = (profile->jerk_dec*T_dec / (2.0f*M_PI_F));
	profile->vel_equation_constants[3] = (profile->jerk_dec*T_dec*T_dec/(4.0f*M_PI_F*M_PI_F));

	profile->acc_equation_constants[0] = (profile->jerk_acc*T_acc / (2.0f*M_PI_F));
	profile->acc_equation_constants[1] = (profile->jerk_dec*T_dec / (2.0f*M_PI_F));

}


void sinusoidal_profile_update(sinusoidal_profile *profile, int64_t n, float fs)
{
	float pos_ref = profile->initial_position;
	float vel_ref = 0.0f;
	float acc_ref = 0.0f;
	float jerk_ref = 0.0f;

	const float j_acc = profile->jerk_acc;
	const float j_dec = profile->jerk_dec;
	const float t = (float)n / fs;

	const float t1 = profile->t1;
	const float t2 = profile->t2;
	const float v_max = profile->max_velocity;

	const float omega_acc = profile->omega_acc;
	const float omega_dec = profile->omega_dec;

	if(n > profile->deaccel_samples)
	{
		pos_ref = profile->target_position;

		vel_ref = 0.0f;

		acc_ref = 0.0f;

		jerk_ref = 0.0f;

		n = profile->deaccel_samples + 1;
	}
	else if(n > profile->noaccel_samples)
	{
		const float tao = t - t2;
		float cos_dec = 0.0f;
		float sin_dec = 0.0f;
		profile_sin_cos(omega_dec * tao, &cos_dec, &sin_dec);

		pos_ref +=  profile->dec_offset	+
				(tao * v_max) 						 			-
				profile->pos_equation_constants[2] * tao*tao 	+
				profile->pos_equation_constants[3] * (1.0f - cos_dec);
		vel_ref = v_max 						  				-
				profile->vel_equation_constants[2] * tao 	  	+
				profile->vel_equation_constants[3] * sin_dec;

		acc_ref = profile->acc_equation_constants[1] * (cos_dec - 1.0f);

		jerk_ref = -j_dec * sin_dec;
	}
	else if (n > profile->accel_samples)
	{
		const float tao = t - t1;
		pos_ref += profile->noacc_offset +
				tao * v_max;

		vel_ref = v_max;

		acc_ref = 0.0f;

		jerk_ref = 0.0f;
	}
	else
	{
		float cos_acc = 0.0f;
		float sin_acc = 0.0f;
		profile_sin_cos(omega_acc * t, &cos_acc, &sin_acc);
		pos_ref += profile->pos_equation_constants[0] *t*t -
				profile->pos_equation_constants[1] * (1.0f - cos_acc);

		vel_ref = profile->vel_equation_constants[0] * t -
				profile->vel_equation_constants[1] * sin_acc;

		acc_ref = profile->acc_equation_constants[0] * (1.0f - cos_acc);

		jerk_ref = j_acc * sin_acc;
	}

	profile->pos_ref = pos_ref;
	profile->vel_ref = vel_ref;
	profile->acc_ref = acc_ref;
	profile->jerk_ref = jerk_ref;

	profile->current_samples = n;
}

float sinusoidal_profile_get_position_ref(sinusoidal_profile *profile, int64_t n, float fs)
{
	float pos_ref = profile->initial_position;

	const float t = (float)n / fs;

	const float t1 = profile->t1;
	const float t2 = profile->t2;
	const float T_acc = profile->T_acc;
	const float T_dec = profile->T_dec;
	const float v_max = profile->max_velocity;

	if(n > profile->deaccel_samples)
	{
		pos_ref = profile->target_position;
	}
	else if(n > profile->noaccel_samples)
	{
		float tao = t - t2;
		float cos_dec = profile_cos(2.0f*M_PI_F*tao / T_dec);
		pos_ref +=  profile->dec_offset						+
				(tao * v_max) 						 	 	-
				profile->pos_equation_constants[2] * tao*tao 	+
				profile->pos_equation_constants[3] * (1.0f - cos_dec);
	}
	else if (n > profile->accel_samples)
	{
		float tao = t - t1;
		pos_ref += profile->noacc_offset +
				tao * v_max;
	}
	else
	{
		float cos_acc = profile_cos(2.0f*M_PI_F*t / T_acc);
		pos_ref += profile->pos_equation_constants[0] *t*t -
				profile->pos_equation_constants[1] * (1.0f - cos_acc);
	}

	return pos_ref;
}

float sinusoidal_profile_get_velocity_ref(sinusoidal_profile *profile, int64_t n, float fs)
{
	float vel_ref = 0.0f;

	const float j_acc = profile->jerk_acc;
	const float j_dec = profile->jerk_dec;
	const float t = (float)n / fs;

	const float t2 = profile->t2;
	const float T_acc = profile->T_acc;
	const float T_dec = profile->T_dec;
	const float v_max = profile->max_velocity;

	if(n > profile->deaccel_samples)
	{
		vel_ref = 0.0;
	}
	else if(n > profile->noaccel_samples)
	{
		float tao = t - t2;
		vel_ref = v_max 						  -
				(j_dec*T_dec/(2.0f*M_PI_F))*tao 	  +
				(j_dec*T_dec*T_dec/(4.0f*M_PI_F*M_PI_F))*profile_sin(2.0f*M_PI_F*tao / T_dec);
	}
	else if (n > profile->accel_samples)
	{
		vel_ref = v_max;
	}
	else
	{
		vel_ref = (j_acc*T_acc/(2.0f*M_PI_F))*t -
				(j_acc*T_acc*T_acc/(4.0f*M_PI_F*M_PI_F))*profile_sin(2.0f*M_PI_F*t / T_acc);
	}
	return vel_ref;
}

float sinusoidal_profile_get_acceleration_ref(sinusoidal_profile *profile, int64_t n, float fs)
{
	float acc_ref = 0.0f;

	const float j_acc = profile->jerk_acc;
	const float j_dec = profile->jerk_dec;
	const float t = (float)n / fs;

	const float t2 = profile->t2;
	const float T_acc = profile->T_acc;
	const float T_dec = profile->T_dec;

	if(n > profile->deaccel_samples)
	{
		acc_ref = 0.0;
	}
	else if(n > profile->noaccel_samples)
	{
		float tao = t - t2;
		float cos_dec = profile_cos(2.0f*M_PI_F*tao / T_dec);
		acc_ref = (j_dec*T_dec/(2.0f*M_PI_F))*(cos_dec - 1.0f);
	}
	else if (n > profile->accel_samples)
	{
		acc_ref = 0.0f;
	}
	else
	{
		float cos_acc = profile_cos(2.0f*M_PI_F*t / T_acc);
		acc_ref = (j_acc*T_acc/(2.0f*M_PI_F))*(1.0f - cos_acc);
	}

	return acc_ref;
}

float sinusoidal_profile_get_jerk_ref(sinusoidal_profile *profile, int64_t n, float fs)
{
	float jerk_ref = 0.0f;

	const float j_acc = profile->jerk_acc;
	const float j_dec = profile->jerk_dec;
	const float t = (float)n / fs;

	const float t2 = profile->t2;
	const float T_acc = profile->T_acc;
	const float T_dec = profile->T_dec;

	if(n > profile->deaccel_samples)
	{
		jerk_ref = 0.0f;
	}
	else if(n > profile->noaccel_samples)
	{
		float tao = t - t2;
		jerk_ref = -j_dec * profile_sin(2.0f*M_PI_F*tao / T_dec);

	}
	else if (n > profile->accel_samples)
	{
		jerk_ref = 0.0;
	}
	else
	{
		jerk_ref = j_acc * profile_sin(2.0f*M_PI_F*t / T_acc);
	}

	return jerk_ref;
}



