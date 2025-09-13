/*
 * stepper.h
 *
 *  Created on: Aug 21, 2025
 *      Author: alper
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include <stdbool.h>

#include "trapezoidal_profile.h"
#include "sinusoidal_profile.h"
#include "lowpass_filters.h"
#include "controller_params.h"

typedef struct
{

	volatile uint32_t pwm_pin;
	volatile uint32_t dir_pin;

	volatile uint32_t pwm_slice;
	volatile bool swap_dir;

	volatile int64_t min_position_boundary;
	volatile int64_t max_position_boundary;

	volatile float max_pwm_hz;

	volatile int32_t velocity;
	volatile float velocity_filtered;
	volatile float pwm_hz;

	volatile int32_t position;
	volatile int32_t prev_counter;

	volatile float prev_pwm_hz;

	int32_t fs;

	trapezoidal_profile trapezoidal;
	volatile bool trapezoidal_set;
	sinusoidal_profile sinusoidal;
	volatile bool sinusoidal_set;

	pid_params pid;
	feedforward_params ff;

	butter2_filter filter;

} stepper;

bool stepper_init(stepper *motor);
bool stepper_pid_init(stepper *motor,
		int64_t min_pos_boundary, int64_t max_pos_boundary,
		float kp, float ki, float kd,
		float p_bound, float i_bound, float d_bound,
		float fc, float fs
);

bool stepper_pid_ff_init(stepper *motor,
		int64_t min_pos_boundary, int64_t max_pos_boundary,
		float kp, float ki, float kd,
		float kv, float ka, float v_off,
		float p_bound, float i_bound, float d_bound,
		float fc, float fs
);

void stepper_update(stepper *motor, uint32_t control_hz);
void stepper_update_filtered(stepper* motor);

bool stepper_dir_get(stepper* motor);
void stepper_dir_set(stepper* motor, bool dir);

float stepper_pwm_hz_set(stepper* motor, float hz);

float stepper_speed_set(stepper* motor, float speed);

void stepper_current_position_set(stepper* motor, int32_t pos);

void stepper_position_pid_control(stepper* motor, int32_t ref, float epsilon);
void stepper_velocity_pid_control(stepper* motor, float ref, float epsilon);

void stepper_flexible_profile_set(stepper *motor,
		int64_t target_pos,
		float max_velocity, float final_velocity,
		float max_accel, float max_deaccel,
		float jerk_acc, float jerk_dec);
void stepper_flexible_profile_update(stepper *motor);

void stepper_trapezoidal_profile_set(stepper *motor,
		int64_t target_pos,
		float max_velocity, float final_velocity,
		float max_accel, float max_deaccel);
void stepper_trapezoidal_profile_update(stepper *motor);

void stepper_sinusoidal_profile_set(stepper *motor,
		int64_t target_pos,
		float max_velocity,
		float jerk_acc, float jerk_dec);
void stepper_sinusoidal_profile_update(stepper *motor);


#endif /* INC_STEPPER_H_ */
