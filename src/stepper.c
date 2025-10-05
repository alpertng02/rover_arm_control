/*
 * stepper.c
 *
 *  Created on: Aug 21, 2025
 *      Author: alper
 */

#include "stepper.h"


#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/platform.h"
#include "pico/time.h"

#include <stdlib.h>
#include <math.h>

#define CLAMP(x, a, b) (((x) < (a)) ? (a) : \
			((b) < (x)) ? (b) : (x))

 // All indexes in arrays below represent the slice number of the corresponding
 // stepper. Unfortunately, PWM interrupt can only be used with global variables
 // so in order to protect the encapsulation of the class, all global variables
 // are static.
static volatile uint stepper_count = 0;
static volatile uint stepper_slices[8];
static volatile int32_t stepper_dir_pins[8];
static volatile int32_t stepper_pwm_counters[8];

/**
 * @brief PWM wrap interrupt to keep track of steppers' position and
 * automatically stop them once the position has been reached.
 * Due to the hardware limitations on RP2040, all PWM slices share the same
 * callback so we need to handle each stepper in this single function.
 *
 */
static void stepper_pwm_counter_callback(void) {
	const uint32_t irq = pwm_get_irq_status_mask();
	for (int i = 0; i < 8; i++) {
		const uint slice = i;
		if (irq & (1 << slice)) {
			// Clear the interrupt flag so the interrupt does not trigger again.
			pwm_clear_irq(slice);
			// Update the position.
			int32_t dir = gpio_get(stepper_dir_pins[slice]) ? 1 : -1;
			stepper_pwm_counters[slice] += dir;
		}
	}
}

void stepper_dir_init(uint32_t dir)
{
  // Initialize the direction pin.
	gpio_init(dir);
	gpio_set_dir(dir, true);
	gpio_put(dir, true);
}

void stepper_pwm_init(uint32_t pin, uint32_t slice) {
	gpio_set_function(pin, GPIO_FUNC_PWM);

	pwm_config config = pwm_get_default_config();

	uint32_t clock_hz = clock_get_hz(clk_sys);

	float clock_div = (float) clock_hz / (0x0001 << 19);
	while (clock_div > (255.0f + 15.0f / 16.0f)) {
		clock_div /= 2.0f;
	}
	const uint16_t wrap = UINT16_MAX;

	pwm_config_set_clkdiv(&config, clock_div);
	pwm_config_set_wrap(&config, wrap);

	uint32_t pwm_clock_hz = (uint32_t) (clock_hz / clock_div);

	pwm_init(slice, &config, false);
	pwm_set_irq_enabled(slice, true);
	pwm_set_enabled(slice, false);

	if (irq_get_exclusive_handler(PWM_IRQ_WRAP) == NULL) {
		irq_set_exclusive_handler(PWM_IRQ_WRAP, stepper_pwm_counter_callback);
		irq_set_priority(PWM_IRQ_WRAP, PICO_HIGHEST_IRQ_PRIORITY);
		irq_set_enabled(PWM_IRQ_WRAP, true);
	}
}

bool stepper_init(stepper* motor) {
	motor->position = 0;
	motor->velocity = 0;
	motor->velocity_filtered = 0;

	motor->pwm_slice = pwm_gpio_to_slice_num(motor->pwm_pin);

	stepper_slices[stepper_count] = motor->pwm_slice;
	stepper_pwm_counters[motor->pwm_slice] = 0;
	stepper_dir_pins[motor->pwm_slice] = motor->dir_pin;
	stepper_count++;

	stepper_dir_init(motor->dir_pin);
	stepper_pwm_init(motor->pwm_pin, motor->pwm_slice);

	return true;
}

void stepper_enable(stepper* motor, bool enable) {
	pwm_set_enabled(motor->pwm_slice, enable);
}

void stepper_dir_set(stepper* motor, bool dir)
{
	gpio_put(motor->dir_pin, dir);
}

bool stepper_dir_get(stepper* motor)
{
	return gpio_get(motor->dir_pin);
}

void stepper_current_position_set(stepper* motor, int32_t pos)
{
	motor->position = pos;
	motor->prev_counter = pos;
	stepper_pwm_counters[motor->pwm_slice] = pos;
}


bool stepper_pid_init(stepper* motor,
	int64_t min_pos_boundary, int64_t max_pos_boundary,
	float kp, float ki, float kd,
	float p_bound, float i_bound, float d_bound,
	float fc, float fs
) {

	butter2_filter_init(&(motor->filter), fc, fs);

	pid_params_init(&(motor->pid), kp, ki, kd,
		p_bound, i_bound, d_bound, fs);
	bool motor_ok = stepper_init(motor);

	motor->min_position_boundary = min_pos_boundary;
	motor->max_position_boundary = max_pos_boundary;
	motor->fs = fs;

	return motor_ok;
}

bool stepper_pid_ff_init(stepper* motor,
	int64_t min_pos_boundary, int64_t max_pos_boundary,
	float kp, float ki, float kd,
	float kv, float ka, float v_off,
	float p_bound, float i_bound, float d_bound,
	float fc, float fs
) {
	feedforward_params_init(&(motor->ff), kv, ka, 0.0f, v_off);

	bool motor_ok = stepper_pid_init(motor,
		min_pos_boundary, max_pos_boundary,
		kp, ki, kd,
		p_bound, i_bound, d_bound,
		fc, fs
	);

	return motor_ok;
}

// Returns the actual frequency achieved
float stepper_pwm_hz_set(stepper* motor, float hz) {
    if (hz <= 0.0f) hz = 1.0f;

    uint32_t sys_clk = clock_get_hz(clk_sys); // usually 125 MHz
    float clkdiv;
    uint32_t wrap;

    // First try with wrap=1 (max frequency approach)
    clkdiv = (float)sys_clk / (hz * 2.0f);

    if (clkdiv >= 1.0f && clkdiv <= 255.0f) {
        // Perfect: wrap = 1
        wrap = 1;
    } else {
        // Clamp clkdiv, compute wrap instead
        if (clkdiv < 1.0f) clkdiv = 1.0f;
        if (clkdiv > 255.0f) clkdiv = 255.0f;
        wrap = (uint32_t)((float)sys_clk / (clkdiv * hz)) - 1;
        if (wrap < 1) wrap = 1;
        if (wrap > 65535) wrap = 65535;
    }

    pwm_set_clkdiv(motor->pwm_slice, clkdiv);
    pwm_set_wrap(motor->pwm_slice, wrap);

    // 50% duty cycle
    pwm_set_gpio_level(motor->pwm_pin, wrap / 2);

    // Calculate the actual frequency achieved
    float hz_actual = (float)sys_clk / (clkdiv * (wrap + 1));

    return hz_actual;
}

float stepper_speed_set(stepper* motor, float speed)
{
	bool dir = speed >= 0.0f ? true : false;
	float hz = fabsf(speed);

	stepper_dir_set(motor, dir);
	// Unachiavable frequency
	if (hz <= 1.0f)
	{
		motor->velocity = 0.0f;
		stepper_enable(motor, false);
		return 0.0f;
	}

	float hz_actual = stepper_pwm_hz_set(motor, hz);
	stepper_enable(motor, true);
	motor->velocity = hz_actual * (dir ? 1.0f : -1.0f);
	return motor->velocity;
}

void stepper_position_pid_control(stepper* motor, int32_t ref, float epsilon) {
	stepper_update_filtered(motor);
	ref = CLAMP(ref, motor->min_position_boundary, motor->max_position_boundary);

	float error = ref - motor->position;
	if (fabsf(error) < epsilon)
	{
		error = 0;
	}
	float speed = pid_params_update_d_filtered(&(motor->pid), error, motor->velocity_filtered);

	stepper_speed_set(motor, speed);
}

void stepper_position_pid_control_ff(stepper* motor,
	float pos_ref,
	float vel_ref,
	float acc_ref,
	float j_ref,
	float epsilon) {
	stepper_update_filtered(motor);
	pos_ref = CLAMP(pos_ref, motor->min_position_boundary, motor->max_position_boundary);

	float error = pos_ref - motor->position;
	if (fabsf(error) < epsilon) {
		error = 0;
	}

	// Normal PID on position error
	float u_pid = pid_params_update_d_filtered(&(motor->pid), error, motor->velocity_filtered);

	// Add feedforward terms
	float u_ff = feedforward_params_update(&(motor->ff), vel_ref, acc_ref, j_ref);

	// Combine
	float speed = (u_pid + u_ff);

	stepper_speed_set(motor, speed);
}

void stepper_velocity_pid_control(stepper* motor, float ref, float epsilon) {
	stepper_update_filtered(motor);
	float error = ref - motor->velocity_filtered;
	if (fabsf(error) < epsilon)
	{
		error = 0;
	}
	float speed = pid_params_update_d_filtered(&(motor->pid), error, motor->velocity_filtered);

	stepper_speed_set(motor, speed);
}

void stepper_update(stepper* motor, uint32_t control_hz) {
	const int32_t temp_counter = stepper_pwm_counters[motor->pwm_slice];
	int32_t velocity = temp_counter - motor->prev_counter;

	motor->position += velocity;
	motor->prev_counter = temp_counter;
}

void stepper_update_filtered(stepper* motor) {
	stepper_update(motor, motor->fs);
	motor->velocity_filtered = butter2_filter_update(&(motor->filter), motor->velocity);
}

void stepper_trapezoidal_profile_set(stepper* motor,
	int64_t target_pos,
	float max_velocity, float final_velocity,
	float max_accel, float max_deaccel) {
	trapezoidal_profile_set_trajectory(&(motor->trapezoidal),
		target_pos, motor->position,
		motor->velocity_filtered, final_velocity, max_velocity,
		max_accel, max_deaccel,
		motor->fs);
	motor->trapezoidal_set = true;
	motor->sinusoidal_set = false;
}

void stepper_trapezoidal_profile_update(stepper* motor) {
	int64_t n = motor->trapezoidal.current_samples + 1;

	trapezoidal_profile_update(&(motor->trapezoidal), n, motor->fs);

	float tolerance = 0.0f;
	stepper_position_pid_control_ff(motor,
		motor->trapezoidal.pos_ref,
		motor->trapezoidal.vel_ref,
		motor->trapezoidal.acc_ref,
		0.0f,
		tolerance);
}


void stepper_sinusoidal_profile_set(stepper* motor, int64_t target_pos, float max_velocity, float max_jerk_acc, float max_jerk_dec) {
	sinusoidal_profile_set_trajectory(&(motor->sinusoidal),
		target_pos, motor->position, max_velocity,
		max_jerk_acc, max_jerk_dec,
		motor->fs);
	motor->trapezoidal_set = false;
	motor->sinusoidal_set = true;
}

void stepper_sinusoidal_profile_update(stepper* motor) {
	int64_t n = motor->sinusoidal.current_samples + 1;

	sinusoidal_profile_update(&(motor->sinusoidal), n, motor->fs);

	const float tolerance = 0.0f;
	stepper_position_pid_control_ff(motor,
		motor->sinusoidal.pos_ref,
		motor->sinusoidal.vel_ref,
		motor->sinusoidal.acc_ref,
		motor->sinusoidal.jerk_ref,
		tolerance);

}


void stepper_flexible_profile_set(stepper* motor,
	int64_t target_pos,
	float max_velocity, float final_velocity,
	float max_accel, float max_deaccel,
	float jerk_acc, float jerk_dec) {
	const float zero_velocity_threshold = 10.0f;

	if (fabsf(motor->velocity_filtered) < zero_velocity_threshold && fabsf(final_velocity) <= 1.0f)
	{
		stepper_sinusoidal_profile_set(motor, target_pos, max_velocity, jerk_acc, jerk_dec);
	} else
	{
		stepper_trapezoidal_profile_set(motor, target_pos, max_velocity, final_velocity, max_accel, max_deaccel);
	}
}

void stepper_flexible_profile_update(stepper* motor) {
	if (motor->sinusoidal_set && !motor->trapezoidal_set)
	{
		stepper_sinusoidal_profile_update(motor);
	} else if (motor->trapezoidal_set && !motor->sinusoidal_set)
	{
		stepper_trapezoidal_profile_update(motor);
	} else if (!motor->sinusoidal_set && !motor->trapezoidal_set)
	{
		stepper_update_filtered(motor);
	}
}



