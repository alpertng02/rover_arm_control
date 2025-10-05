/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <math.h>
#include <stdbool.h>

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/timer.h"
#include "tusb.h"

#include "manipulator_packets.h"
#include "usb_cdc.h"

#include "stepper.h"
#include "l298n.h"

#define STEPPER0_PUL_PIN 			(2)
#define STEPPER1_PUL_PIN 			(6)
#define STEPPER2_PUL_PIN 			(8)

#define STEPPER0_DIR_PIN 			(3)
#define STEPPER1_DIR_PIN 			(7)
#define STEPPER2_DIR_PIN 			(9)

#define GRIPPER_MOTOR0_LPWM_PIN 	(14)
#define GRIPPER_MOTOR1_LPWM_PIN 	(10)
#define GRIPPER_MOTOR2_LPWM_PIN 	(12)

#define GRIPPER_MOTOR0_RPWM_PIN 	(15)
#define GRIPPER_MOTOR1_RPWM_PIN 	(11)
#define GRIPPER_MOTOR2_RPWM_PIN 	(13)

#define MOTOR_COUNT				 	    (3)

#define DEFAULT_MIN_POSITION_BOUNDARY	(-50000)
#define DEFAULT_MAX_POSITION_BOUNDARY   (50000)

#define DEFAULT_MOTOR_MAX_DUTY 			(80.0f)
#define DEFAULT_MOTOR_MAX_VELOCITY 		(20000.0f)

#define DEFAULT_CONTROL_FREQUENCY_HZ 	(1000.0f)
#define DEFAULT_FEEDBACK_FREQUENCY_HZ   (200.0f)
#define DEFAULT_LOWPASS_CUTOFF_HZ    	(100.0f)

#define DEFAULT_P_BOUND 				(10000.0f)
#define DEFAULT_I_BOUND 				(5000.0f)
#define DEFAULT_D_BOUND 				(5000.0f)

#define DEFAULT_KP 						(DEFAULT_P_BOUND / DEFAULT_MOTOR_MAX_VELOCITY)
#define DEFAULT_KI 						(DEFAULT_KP / 100.0f)
#define DEFAULT_KD 						(DEFAULT_KP / 10000.0f)

#define DEFAULT_KV 						(1.0f)
#define DEFAULT_KA 						(DEFAULT_KV / 1000.0f)
#define DEFAULT_KJ 						(DEFAULT_KV / 1000000.0f)

#define MOTOR_PID_TOLERANCE			    (0.0f)


static volatile bool device_is_running = false;
static volatile bool device_init_mode_enabled = false;

static volatile float motor_control_frequency_hz = DEFAULT_CONTROL_FREQUENCY_HZ;
static volatile float motor_control_feedback_hz = DEFAULT_FEEDBACK_FREQUENCY_HZ;
static volatile float motor_filter_cutoff_hz = DEFAULT_LOWPASS_CUTOFF_HZ;

static volatile float motor_kp = DEFAULT_KP;
static volatile float motor_ki = DEFAULT_KI;
static volatile float motor_kd = DEFAULT_KD;
static volatile float motor_p_bound = DEFAULT_P_BOUND;
static volatile float motor_i_bound = DEFAULT_I_BOUND;
static volatile float motor_d_bound = DEFAULT_D_BOUND;

static ManipulatorFeedbackPacket feedback = {};
static ManipulatorFeedbackPacket feedback_to_send = {};

static repeating_timer_t motor_control_timer;

static float motor_target_velocities[MOTOR_COUNT] = { 0.0f };

static volatile uint64_t loop_count = 0;
static volatile bool print_ready = false;

static volatile bool send_state_packet_mode = true;

static ManipulatorStatePacket state_pkt = {
	.device_id = MANIPULATOR_DEVICE_ID
};

static ManipulatorInitPacket init_configuration = {

	.overwrite_pinout = false,

	.joint_dir_pins = { STEPPER0_DIR_PIN, STEPPER1_DIR_PIN, STEPPER2_DIR_PIN },
	.joint_pul_pins = { STEPPER0_PUL_PIN, STEPPER1_PUL_PIN, STEPPER2_PUL_PIN },

	.gripper_lpwm_pins = { GRIPPER_MOTOR0_LPWM_PIN, GRIPPER_MOTOR1_LPWM_PIN, GRIPPER_MOTOR2_LPWM_PIN },
	.gripper_rpwm_pins = { GRIPPER_MOTOR0_RPWM_PIN, GRIPPER_MOTOR1_RPWM_PIN, GRIPPER_MOTOR2_RPWM_PIN },

	.joint_swap_dirs = { false, false, false },
	.gripper_swap_dirs = { false, false, false },

	.joint_initial_pos = { 0, 0, 0 },

	.max_joint_pos_boundaries = { DEFAULT_MAX_POSITION_BOUNDARY, DEFAULT_MAX_POSITION_BOUNDARY, DEFAULT_MAX_POSITION_BOUNDARY },
	.min_joint_pos_boundaries = { DEFAULT_MIN_POSITION_BOUNDARY, DEFAULT_MIN_POSITION_BOUNDARY, DEFAULT_MIN_POSITION_BOUNDARY },

	.max_dutycycle = DEFAULT_MOTOR_MAX_DUTY,
	.max_velocity = DEFAULT_MOTOR_MAX_VELOCITY,
	.lowpass_fc = DEFAULT_LOWPASS_CUTOFF_HZ,
	.kp = DEFAULT_KP,
	.ki = DEFAULT_KI,
	.kd = DEFAULT_KD,
	.p_bound = DEFAULT_P_BOUND,
	.i_bound = DEFAULT_I_BOUND,
	.d_bound = DEFAULT_D_BOUND,
	.feedback_hz = DEFAULT_FEEDBACK_FREQUENCY_HZ,
	.control_hz = DEFAULT_CONTROL_FREQUENCY_HZ
};

static stepper joint_motors[MOTOR_COUNT] =
{

	[0] = {
		.dir_pin = STEPPER0_DIR_PIN,
		.pwm_pin = STEPPER0_PUL_PIN
},

[1] = {
		.dir_pin = STEPPER1_DIR_PIN,
		.pwm_pin = STEPPER1_PUL_PIN
},

[2] = {
		.dir_pin = STEPPER2_DIR_PIN,
		.pwm_pin = STEPPER2_PUL_PIN
}
};

static l298n gripper_motors[MOTOR_COUNT] =
{

	[0] = {
		.pwml_pin = GRIPPER_MOTOR0_LPWM_PIN,
		.pwmr_pin = GRIPPER_MOTOR0_RPWM_PIN
},
[1] = {
		.pwml_pin = GRIPPER_MOTOR1_LPWM_PIN,
		.pwmr_pin = GRIPPER_MOTOR1_RPWM_PIN
},
[2] = {
		.pwml_pin = GRIPPER_MOTOR2_LPWM_PIN,
		.pwmr_pin = GRIPPER_MOTOR2_RPWM_PIN
},

};

void joint_stepper_control_loop(int32_t motor_count) {

	for (int i = 0; i < motor_count; i++) {
		if (device_is_running) {
			stepper_flexible_profile_update(&(joint_motors[i]));
		} else {
			joint_motors[i].sinusoidal_set = false;
			joint_motors[i].trapezoidal_set = false;
			stepper_speed_set(&joint_motors[i], 0.0f);
			stepper_update(&joint_motors[i], joint_motors[i].fs);
		}
	}
	if (loop_count % (uint64_t) (init_configuration.control_hz / init_configuration.feedback_hz) == 0)
	{
		for (int i = 0; i < MOTOR_COUNT; i++) {
			feedback.joint_positions[i] = joint_motors[i].position;
			feedback.joint_velocities[i] = joint_motors[i].velocity_filtered;
			feedback.gripper_pwm_duties[i] = gripper_motors[i].pwm_duty;
		}
		memcpy(&feedback_to_send, &feedback, sizeof(ManipulatorFeedbackPacket));
		print_ready = true;
	}

	loop_count++;
}


bool stepper_control_timer_interrupt(repeating_timer_t* rt) {

	joint_stepper_control_loop(MOTOR_COUNT);

	return true;
}

bool manipulator_init_packet_handler(ManipulatorInitPacket* pkt) {
	if (!pkt) {
		return false;
	}

	cancel_repeating_timer(&motor_control_timer);

	for (int i = 0; i < MOTOR_COUNT; i++) {
		if (pkt->overwrite_pinout == 1) {

			joint_motors[i].dir_pin = pkt->joint_dir_pins[i];
			joint_motors[i].pwm_pin = pkt->joint_pul_pins[i];

			gripper_motors[i].pwml_pin = pkt->gripper_lpwm_pins[i];
			gripper_motors[i].pwmr_pin = pkt->gripper_rpwm_pins[i];

			init_configuration.joint_dir_pins[i] = pkt->joint_dir_pins[i];
			init_configuration.joint_pul_pins[i] = pkt->joint_pul_pins[i];

			init_configuration.gripper_lpwm_pins[i] = pkt->gripper_lpwm_pins[i];
			init_configuration.gripper_rpwm_pins[i] = pkt->gripper_rpwm_pins[i];
		}
		if (pkt->gripper_swap_dirs[i]) {
			gripper_motors[i].pwml_pin = init_configuration.gripper_rpwm_pins[i];
			gripper_motors[i].pwmr_pin = init_configuration.gripper_lpwm_pins[i];
		}
		init_configuration.joint_swap_dirs[i] = pkt->joint_swap_dirs[i];
		init_configuration.gripper_swap_dirs[i] = pkt->gripper_swap_dirs[i];

		init_configuration.max_dutycycle = pkt->max_dutycycle;
		init_configuration.max_velocity = pkt->max_velocity;

		init_configuration.joint_initial_pos[i] = pkt->joint_initial_pos[i];

		init_configuration.max_joint_pos_boundaries[i] = pkt->max_joint_pos_boundaries[i];
		init_configuration.min_joint_pos_boundaries[i] = pkt->min_joint_pos_boundaries[i];
	}

	init_configuration.lowpass_fc = pkt->lowpass_fc;

	init_configuration.kp = pkt->kp;
	init_configuration.ki = pkt->ki;
	init_configuration.kd = pkt->kd;

	init_configuration.p_bound = pkt->p_bound;
	init_configuration.i_bound = pkt->i_bound;
	init_configuration.d_bound = pkt->d_bound;

	init_configuration.kv = pkt->kv;
	init_configuration.ka = pkt->ka;
	init_configuration.kj = pkt->kj;


	init_configuration.feedback_hz = pkt->feedback_hz;
	init_configuration.control_hz = pkt->control_hz;

	for (int i = 0; i < MOTOR_COUNT; i++) {
		stepper_pid_ff_init(&joint_motors[i],
			pkt->min_joint_pos_boundaries[i], pkt->max_joint_pos_boundaries[i],
			pkt->kp, pkt->ki, pkt->kd,
			pkt->kv, pkt->ka, 1.0f,
			pkt->p_bound, pkt->i_bound, pkt->d_bound,
			pkt->lowpass_fc, pkt->control_hz);
		stepper_current_position_set(&joint_motors[i], pkt->joint_initial_pos[i]);

		l298n_init(&gripper_motors[i]);
	}

	add_repeating_timer_us(
		(int64_t) (-1000000 / init_configuration.control_hz),
		stepper_control_timer_interrupt,
		NULL,
		&motor_control_timer
	);

	device_is_running = true;
	device_init_mode_enabled = false;
	send_state_packet_mode = false;

	return true;
}

void manipulator_command_packet_handler(ManipulatorCommandPacket* pkt) {
	switch ((ManipulatorCommands) pkt->command_id) {

	case set_joint_velocity:
		for (int i = 0; i < MOTOR_COUNT; i++) {
			stepper_speed_set(&joint_motors[i], pkt->content.joint_velocity.velocities[i]);
		}
		break;
	case set_joint_trajectory:
		for (int i = 0; i < MOTOR_COUNT; i++) {
			stepper_flexible_profile_set(&joint_motors[i],
				pkt->content.joint_trajectory.target_positions[i],
				pkt->content.joint_trajectory.maximum_velocities[i],
				pkt->content.joint_trajectory.final_velocities[i],
				pkt->content.joint_trajectory.accelerations[i],
				pkt->content.joint_trajectory.deaccelerations[i],
				pkt->content.joint_trajectory.acc_jerks[i],
				pkt->content.joint_trajectory.dec_jerks[i]);
		}
		break;

	case set_gripper_duties:
		for (int i = 0; i < 3; i++) {
			l298n_speed_set(&gripper_motors[i], pkt->content.gripper_duty.gripper_duties[i]);
		}
		break;

	case set_joint_pos_boundaries:
		for (int i = 0; i < MOTOR_COUNT; i++) {
			joint_motors[i].max_position_boundary = (int32_t)pkt->content.joint_pos_boundaries.max_pos;
			joint_motors[i].min_position_boundary = (int32_t)pkt->content.joint_pos_boundaries.min_pos;

			init_configuration.max_joint_pos_boundaries[i] = (int32_t)pkt->content.joint_pos_boundaries.max_pos;
			init_configuration.min_joint_pos_boundaries[i] = (int32_t)pkt->content.joint_pos_boundaries.min_pos;
		}
		break;

	case set_joint_current_pos:
		for (int i = 0; i < MOTOR_COUNT; i++) {
			init_configuration.joint_initial_pos[i] = pkt->content.joint_current_pos.current_pos[i];
			stepper_current_position_set(&joint_motors[i], pkt->content.joint_current_pos.current_pos[i]);
		}
		break;

	case set_pid_params:

		for (int i = 0; i < MOTOR_COUNT; i++) {
			joint_motors[i].pid.kp = pkt->content.pid_params.kp;
			joint_motors[i].pid.ki = pkt->content.pid_params.ki;
			joint_motors[i].pid.kd = pkt->content.pid_params.kd;
			joint_motors[i].pid.i = 0.0f;
		}
		init_configuration.kp = pkt->content.pid_params.kp;
		init_configuration.ki = pkt->content.pid_params.ki;
		init_configuration.kd = pkt->content.pid_params.kd;
		break;

	case set_pid_bounds:
		for (int i = 0; i < MOTOR_COUNT; i++) {
			joint_motors[i].pid.p_bound = pkt->content.pid_bounds.p_bound;
			joint_motors[i].pid.i_bound = pkt->content.pid_bounds.i_bound;
			joint_motors[i].pid.d_bound = pkt->content.pid_bounds.d_bound;
		}
		init_configuration.p_bound = pkt->content.pid_bounds.p_bound;
		init_configuration.i_bound = pkt->content.pid_bounds.i_bound;
		init_configuration.d_bound = pkt->content.pid_bounds.d_bound;
		break;

	case set_ff_params:
		for (int i = 0; i < MOTOR_COUNT; i++) {
			joint_motors[i].ff.kv = pkt->content.ff_params.ff_vel_gain;
			joint_motors[i].ff.ka = pkt->content.ff_params.ff_acc_gain;
			joint_motors[i].ff.kj = pkt->content.ff_params.ff_jerk_gain;
		}
		init_configuration.kv = pkt->content.ff_params.ff_vel_gain;
		init_configuration.ka = pkt->content.ff_params.ff_acc_gain;
		init_configuration.kj = pkt->content.ff_params.ff_jerk_gain;
		break;

	case set_lowpass_fc:
		for (int i = 0; i < MOTOR_COUNT; i++) {
			butter2_filter_init(&(joint_motors[i].filter), pkt->content.lowpass.lowpass_fc, joint_motors[i].fs);
		}
		init_configuration.lowpass_fc = pkt->content.lowpass.lowpass_fc;
		break;

	case set_max_velocity:
		init_configuration.max_velocity = pkt->content.max_velocity.max_velocity;
		for (int i = 0; i < MOTOR_COUNT; i++) {
			joint_motors[i].max_pwm_hz = pkt->content.max_velocity.max_velocity;
		}
		break;

	case set_feedback_hz:
		init_configuration.feedback_hz = pkt->content.feedback_hz.feedback_hz;
		break;
	case set_control_hz:
		init_configuration.control_hz = pkt->content.control_hz.control_hz;
		break;

	case running_mode_enable:
		if (pkt->content.mode_enable.enable) {
			device_is_running = true;
		} else {
			device_is_running = false;
		}
		break;
	case init_mode_enable:
		if (pkt->content.mode_enable.enable)
			device_init_mode_enabled = true;
		else
			device_init_mode_enabled = false;
		break;
	case send_device_state:
		state_pkt.init_mode_enabled = device_init_mode_enabled;
		state_pkt.is_running = device_is_running;
		state_pkt.device_id = MANIPULATOR_DEVICE_ID;
		send_state_packet_mode = true;

		usb_cdc_send_manipulator_state_packet(&state_pkt);

		break;

	default:
		// Unknown command
		break;
	}
}

int main(void) {

	stdio_init_all();
	while (!tud_cdc_connected()) { sleep_ms(10); }

	while (tud_cdc_connected()) {

		ManipulatorCommandPacket command_received = {};

		if (device_init_mode_enabled) {
			state_pkt.init_mode_enabled = device_init_mode_enabled;
			state_pkt.is_running = device_is_running;
			state_pkt.device_id = MANIPULATOR_DEVICE_ID;

			usb_cdc_send_manipulator_state_packet(&state_pkt);

			ManipulatorInitPacket init_packet = {};
			if (usb_cdc_receive_manipulator_init_packet(&init_packet, (uint32_t) (1000 / init_configuration.feedback_hz))) {
				manipulator_init_packet_handler(&init_packet);
			}

		} else {
			if (usb_cdc_receive_manipulator_command_packet(&command_received, (uint32_t) (1000 / init_configuration.feedback_hz))) {
				manipulator_command_packet_handler(&command_received);
			}


			if (print_ready) {
				usb_cdc_send_manipulator_feedback_packet(&feedback_to_send);
				print_ready = false;
			}
		}


	}

	for (int i = 0; i < MOTOR_COUNT; i++) {
		stepper_speed_set(&joint_motors[i], 0.0f);
		l298n_speed_set(&gripper_motors[i], 0.0f);
	}
}

