/*
 * sinusoidal_profile.h
 *
 *  Created on: Sep 4, 2025
 *      Author: alper
 */

#ifndef INC_SINUSOIDAL_PROFILE_H_
#define INC_SINUSOIDAL_PROFILE_H_

#include <stdint.h>

typedef struct
{
	volatile float target_position;
	volatile float initial_position;

	volatile float max_velocity;
	volatile float jerk_acc;
	volatile float jerk_dec;

	volatile float t1;
	volatile float t2;
	volatile float T_acc;
	volatile float T_dec;
	volatile float noacc_offset;
	volatile float dec_offset;

	volatile float pos_equation_constants[4];
	volatile float vel_equation_constants[4];
	volatile float acc_equation_constants[2];
	volatile float omega_acc;
	volatile float omega_dec;

	volatile int64_t current_samples;
	volatile int64_t accel_samples;
	volatile int64_t deaccel_samples;
	volatile int64_t noaccel_samples;

	volatile float pos_ref;
	volatile float vel_ref;
	volatile float acc_ref;
	volatile float jerk_ref;
} sinusoidal_profile;

void sinusoidal_profile_set_trajectory(sinusoidal_profile *profile,
		float target_pos, float initial_pos,
		float max_velocity,
		float jerk_acc, float jerk_dec,
		float fs);

void sinusoidal_profile_update(sinusoidal_profile *profile, int64_t n, float fs);

float sinusoidal_profile_get_position_ref(sinusoidal_profile *profile, int64_t n, float fs);
float sinusoidal_profile_get_velocity_ref(sinusoidal_profile *profile, int64_t n, float fs);
float sinusoidal_profile_get_acceleration_ref(sinusoidal_profile *profile, int64_t n, float fs);
float sinusoidal_profile_get_jerk_ref(sinusoidal_profile *profile, int64_t n, float fs);

#endif /* INC_SINUSOIDAL_PROFILE_H_ */
