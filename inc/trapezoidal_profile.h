/*
 * trapezoidal_profile.h
 *
 *  Created on: Sep 4, 2025
 *      Author: alper
 */

#ifndef INC_TRAPEZOIDAL_PROFILE_H_
#define INC_TRAPEZOIDAL_PROFILE_H_

#include <stdint.h>

typedef struct
{
	volatile float target_position;
	volatile float initial_position;

	volatile float max_velocity;
	volatile float initial_velocity;
	volatile float final_velocity;

	volatile float max_accel;
	volatile float max_deaccel;

	volatile float t1;
	volatile float t2;
	volatile float noacc_offset;
	volatile float dec_offset;

	volatile int64_t current_samples;
	volatile int64_t accel_samples;
	volatile int64_t deaccel_samples;
	volatile int64_t noaccel_samples;

	volatile float pos_equation_constants[2];
	volatile float vel_equation_constants[2];

	volatile float pos_ref;
	volatile float vel_ref;
	volatile float acc_ref;

} trapezoidal_profile;

void trapezoidal_profile_set_trajectory(trapezoidal_profile *profile, float target_pos, float initial_pos,
		float initial_velocity, float final_vel, float max_velocity,
		float max_accel, float max_deaccel,
		float fs);

void trapezoidal_profile_update(trapezoidal_profile *profile, int64_t n, float fs);

float trapezoidal_profile_get_position_ref(trapezoidal_profile *profile, int64_t n, float fs);
float trapezoidal_profile_get_velocity_ref(trapezoidal_profile *profile, int64_t n, float fs);
float trapezoidal_profile_get_acceleration_ref(trapezoidal_profile *profile, int64_t n, float fs);

#endif /* INC_TRAPEZOIDAL_PROFILE_H_ */
