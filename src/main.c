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

#include "stepper.h"
#include "l298n.h"

#define STEPPER0_PUL_PIN 			(2)
#define STEPPER1_PUL_PIN 			(6)
#define STEPPER2_PUL_PIN 			(8)

#define STEPPER0_DIR_PIN 			(3)
#define STEPPER1_DIR_PIN 			(7)
#define STEPPER2_DIR_PIN 			(9)

#define GRIPPER_MOTOR0_PWM_L_PIN 	(14)
#define GRIPPER_MOTOR1_PWM_L_PIN 	(10)
#define GRIPPER_MOTOR2_PWM_L_PIN 	(12)

#define GRIPPER_MOTOR0_PWM_R_PIN 	(15)
#define GRIPPER_MOTOR1_PWM_R_PIN 	(11)
#define GRIPPER_MOTOR2_PWM_R_PIN 	(13)

#define MOTOR_COUNT				 	(3)

#define MOTOR_CONTROL_FREQ_HZ  		(1000)
#define MOTOR_FILTER_CUTOFF_HZ 		(10)
#define MOTOR_KP 					(1.0f)
#define MOTOR_KI 					(0.01f)
#define MOTOR_KD 					(0.0001f)
#define MOTOR_P_BOUND 				(50000.0f)
#define MOTOR_I_BOUND 				(25000.0f)
#define MOTOR_D_BOUND 				(10000.0f)
#define MOTOR_MIN_STEPS 			(-50000)
#define MOTOR_MAX_STEPS 			(50000)


static char usb_tx_buffer[1024] = { 0 };
static char usb_rx_buffer[1024] = { '\0' };
static int rx_index = 0;

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
		.pwml_pin = GRIPPER_MOTOR0_PWM_L_PIN,
		.pwmr_pin = GRIPPER_MOTOR0_PWM_R_PIN
	},
	[1] = {
		.pwml_pin = GRIPPER_MOTOR1_PWM_L_PIN,
		.pwmr_pin = GRIPPER_MOTOR1_PWM_R_PIN
	},
	[2] = {
		.pwml_pin = GRIPPER_MOTOR2_PWM_L_PIN,
		.pwmr_pin = GRIPPER_MOTOR2_PWM_R_PIN
	},

};

bool usb_cdc_rx_handler(char* buf, int len);

void joint_stepper_control_loop(int32_t motor_count)
{
	int total_bytes = 0;
	for (int i = 0; i < motor_count; i++) {
		stepper_flexible_profile_update(&(joint_motors[i]));

		if (joint_motors[i].sinusoidal_set || joint_motors[i].trapezoidal_set) {
			int pos = (int) joint_motors[i].position;
			int vel = (int) joint_motors[i].velocity_filtered;

			int target_pos = 0;
			int pos_ref = 0;
			int vel_ref = 0;
			int acc_ref = 0;
			int jerk_ref = 0;
			if (joint_motors[i].sinusoidal_set) {
				target_pos = (int) joint_motors[i].sinusoidal.target_position;
				pos_ref = (int) joint_motors[i].sinusoidal.pos_ref;
				vel_ref = (int) joint_motors[i].sinusoidal.vel_ref;
				acc_ref = (int) joint_motors[i].sinusoidal.acc_ref;
				jerk_ref = (int) joint_motors[i].sinusoidal.jerk_ref;
			} else if (joint_motors[i].trapezoidal_set) {
				target_pos = (int) joint_motors[i].trapezoidal.target_position;
				pos_ref = (int) joint_motors[i].trapezoidal.pos_ref;
				vel_ref = (int) joint_motors[i].trapezoidal.vel_ref;
				acc_ref = (int) joint_motors[i].trapezoidal.acc_ref;
				jerk_ref = 0;
			}

			int num_bytes =
				sprintf(usb_tx_buffer + total_bytes * sizeof(char),
					"i:%d,Target:%d,RefPos:%d,Pos:%d,RefVel:%d,Vel:%d,RefAcc:%d,"
					"RefJerk:%d\r\n",
					i, target_pos, pos_ref, pos, vel_ref, vel, acc_ref, jerk_ref);
			total_bytes += num_bytes;
		}
	}
	usb_tx_buffer[total_bytes] = '\0';
	printf(usb_tx_buffer);
}

void gripper_motor_control_loop(int32_t motor_count)
{
	int total_bytes = 0;
	for (int i = 0; i < motor_count; i++) {
		int num_bytes = 0;
		total_bytes += num_bytes;
	}
	usb_tx_buffer[total_bytes] = '\0';
	printf(usb_tx_buffer);

}

bool stepper_control_timer_interrupt(repeating_timer_t* rt) {

	joint_stepper_control_loop(MOTOR_COUNT);

	//gripper_motor_control_loop(MOTOR_COUNT);
	return true;
}


int main(void) {

	stdio_init_all();
	while (!tud_cdc_connected()) { sleep_ms(100); }
	printf("TinyUSB Connected!\n");

	repeating_timer_t stepper_control_timer;
	add_repeating_timer_us(
		-(1000000 / MOTOR_CONTROL_FREQ_HZ),
		stepper_control_timer_interrupt,
		NULL,
		&stepper_control_timer
	);

	for (int i = 0; i < MOTOR_COUNT; i++) {
		stepper_pid_init(&joint_motors[i], MOTOR_MIN_STEPS, MOTOR_MAX_STEPS, MOTOR_KP,
			MOTOR_KI, MOTOR_KD, MOTOR_P_BOUND, MOTOR_I_BOUND,
			MOTOR_D_BOUND, MOTOR_FILTER_CUTOFF_HZ,
			MOTOR_CONTROL_FREQ_HZ);

		l298n_init(&gripper_motors[i]);
	}

	while (true) {
		int c = getchar_timeout_us(1);
		if (c != PICO_ERROR_TIMEOUT) {
			if (c == '\n' || c == '\r') {
				usb_rx_buffer[rx_index] = '\0';
				if (rx_index > 0) {

					if (usb_cdc_rx_handler(usb_rx_buffer, rx_index + 1))
					{
						printf(usb_rx_buffer);
					}
				}
				rx_index = 0;
			} else {
				if (rx_index < (int) (sizeof(usb_rx_buffer) - 1)) {
					usb_rx_buffer[rx_index++] = (char) c;
				}
			}
		}
	}
}

bool usb_cdc_rx_handler(char* buf, int len) {
	if (!buf) {
		return false;
	}

	int i;
	int target_pos;
	int vel_max;
	int vel_final;
	int accel;
	int deaccel;
	int jerk_acc;
	int jerk_dec;

	int parsed = 0;
	parsed = sscanf(
		usb_rx_buffer, "i:%d,Target:%d,VMax:%d,VFinal:%d,Acc:%d,Dec:%d,JAcc:%d,JDec:%d",
		&i, &target_pos, &vel_max, &vel_final, &accel, &deaccel, &jerk_acc, &jerk_dec);
	if (parsed == 8) {
		stepper_flexible_profile_set(&(joint_motors[i]), target_pos, vel_max, vel_final, accel,
			deaccel, jerk_acc, jerk_dec);
		return true;
	}

	parsed = sscanf(usb_rx_buffer,
		"i:%d,Target:%d,VMax:%d,VFinal:%d,Acc:%d,Dec:%d", &i,
		&target_pos, &vel_max, &vel_final, &accel, &deaccel);
	if (parsed == 6) {
		stepper_trapezoidal_profile_set(&(joint_motors[i]), target_pos, vel_max,
			vel_final, accel, deaccel);
		return true;
	}

	parsed = sscanf(usb_rx_buffer, "i:%d,Target:%d,VMax:%d,JAcc:%d,JDec:%d",
		&i, &target_pos, &vel_max, &jerk_acc, &jerk_dec);
	if (parsed == 5) {
		stepper_sinusoidal_profile_set(&(joint_motors[i]), target_pos, vel_max, jerk_acc,
			jerk_dec);
		return true;
	}

	float speed = 0;
	parsed = sscanf(usb_rx_buffer, "i:%d,speed:%f", &i, &speed);
	if (parsed == 2) {
		stepper_speed_set(&(joint_motors[i]), speed);
		return true;
	}

	float gripper_speeds[3] = { 0 };
	parsed = sscanf(usb_rx_buffer, "G1:%f,G2:%f,G3:%f", &gripper_speeds[0], &gripper_speeds[2], &gripper_speeds[2]);
	if (parsed == 3) {
		for (int i = 0; i < 3; i++)
		{
			l298n_speed_set(&gripper_motors[i], gripper_speeds[i]);
		}
		return true;
	}

	float max_pwm_hz = 0;
	parsed = sscanf(usb_rx_buffer, "i:%d,maxPwmHz:%f", &i, &max_pwm_hz);
	if (parsed == 2) {
		joint_motors[i].max_pwm_hz = max_pwm_hz;
		return true;
	}

	int current_pos = 0;
	parsed = sscanf(usb_rx_buffer, "i:%d,pos:%d", &i, &current_pos);
	if (parsed == 2) {
		stepper_current_position_set(&(joint_motors[i]), current_pos);
		return true;
	}

	float fc;
	parsed = sscanf(usb_rx_buffer, "i:%d,fc:%f", &i, &fc);
	if (parsed == 2) {
		butter2_filter_init(&(joint_motors[i].filter), fc, joint_motors[i].fs);
		return true;
	}

	float kp, ki, kd;
	parsed =
		sscanf(usb_rx_buffer, "i:%d,kp:%f,ki:%f,kd:%f", &i, &kp, &ki, &kd);
	if (parsed == 4) {
		joint_motors[i].pid.kp = kp;
		joint_motors[i].pid.ki = ki;
		joint_motors[i].pid.kd = kd;

		joint_motors[i].pid.i = 0;
		return true;
	}

	float p_bound, i_bound, d_bound;
	parsed = sscanf(usb_rx_buffer, "i:%d,pB:%f,iB:%f,dB:%f", &i, &p_bound,
		&i_bound, &d_bound);
	if (parsed == 4) {
		joint_motors[i].pid.p_bound = kp;
		joint_motors[i].pid.i_bound = ki;
		joint_motors[i].pid.d_bound = kd;
		return true;
	}

	float kv, ka, kj, v_off;
	parsed = sscanf(usb_rx_buffer, "i:%d,kv:%f,ka:%f,kj:%f,voff:%f", &i, &kv,
		&ka, &kj, &v_off);
	if (parsed == 5) {
		feedforward_params_init(&(joint_motors[i].ff), kv, ka, kj, v_off);
		return true;
	}

	int min_pos = 0, max_pos = 0;
	parsed = sscanf(usb_rx_buffer, "i:%d,minPos:%d,maxPos:%d", &i, &min_pos,
		&max_pos);
	if (parsed == 3) {
		joint_motors[i].max_position_boundary = max_pos;
		joint_motors[i].min_position_boundary = min_pos;
		return true;
	}

	return false;
}
