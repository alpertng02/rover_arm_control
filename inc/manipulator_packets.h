#ifndef __MANIPULATOR_PACKETS_H__
#define __MANIPULATOR_PACKETS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MANIPULATOR_FRAME_START		(0x6675636B)
#define MANIPULATOR_FRAME_END		(0x676F6F6E)

#define MANIPULATOR_DEVICE_ID		(0x706F726E)	

	typedef enum {
		set_joint_velocity = 0,

		set_joint_trajectory = 1,

		set_gripper_duties = 2,

		set_joint_current_pos = 3,

		set_joint_pos_boundaries = 4,

		set_lowpass_fc = 5,
		set_pid_params = 6,
		set_pid_bounds = 7,

		set_ff_params = 8,

		set_gripper_max_duty = 9,

		set_feedback_hz = 11,
		set_control_hz = 12,

		running_mode_enable = 13,
		init_mode_enable = 14,
		send_device_state = 15

	} ManipulatorCommands;

#pragma pack(push, 1)
	typedef struct {
		uint32_t frame_start;

		uint32_t overwrite_pinout;

		uint8_t joint_dir_pins[3];
		uint8_t joint_pul_pins[3];

		uint8_t gripper_lpwm_pins[3];
		uint8_t gripper_rpwm_pins[3];

		uint8_t joint_swap_dirs[3];
		uint8_t gripper_swap_dirs[3];

		int32_t joint_initial_pos[3];

		int32_t max_joint_pos_boundaries[3];
		int32_t min_joint_pos_boundaries[3];

		float max_dutycycle;
		float lowpass_fc;
		float kp;
		float ki;
		float kd;
		float p_bound;
		float i_bound;
		float d_bound;

		float kv;
		float ka;
		float kj;

		float feedback_hz;
		float control_hz;

		uint16_t padding;

		uint32_t frame_end;
	} ManipulatorInitPacket;
#pragma pack(pop)

#pragma pack(push, 1)
	typedef struct {

		union {
			// 1. Joint velocity commands
			struct {
				float velocities[3];
			} joint_velocity;

			// 2. Joint control commands
			struct {
				int32_t target_positions[3];
				float maximum_velocities[3];
				float final_velocities[3];
				float accelerations[3];
				float deaccelerations[3];
				float acc_jerks[3];
				float dec_jerks[3];
			} joint_trajectory;

			// 3. Gripper duty control
			struct {
				float gripper_duties[3];
			} gripper_duty;

			struct {
				int32_t min_pos[3];
				int32_t max_pos[3];
			} joint_pos_boundaries;

			struct {
				int32_t current_pos[3];
			} joint_current_pos;

			// 4. PID parameters
			struct {
				float kp;
				float ki;
				float kd;
			} pid_params;

			// 5. PID bounds
			struct {
				float p_bound;
				float i_bound;
				float d_bound;
			} pid_bounds;

			// 6. Feedforward parameters
			struct {
				float ff_vel_gain;
				float ff_acc_gain;
				float ff_jerk_gain;
			} ff_params;

			// 7. Lowpass filter cutoff frequency
			struct {
				float lowpass_fc;
			} lowpass;

			// 9. Feedback frequency
			struct {
				float feedback_hz;
			} feedback_hz;

			// 10. Control frequency
			struct {
				float control_hz;
			} control_hz;
			
			// 11. Mode commands (booleans or enable flags)
			struct {
				uint8_t enable;
			} mode_enable;

		};

	} ManipulatorCommandContents;
#pragma pack(pop)

#pragma pack(push, 1)
	typedef struct {
		uint32_t frame_start;
		int32_t command_id;

		ManipulatorCommandContents content;

		uint32_t frame_end;
	} ManipulatorCommandPacket;
#pragma pack(pop)

#pragma pack(push, 1)
	typedef struct {
		uint32_t frame_start;
		int32_t joint_positions[3];
		float joint_velocities[3];
		float gripper_pwm_duties[3];
		uint32_t frame_end;
	} ManipulatorFeedbackPacket;
#pragma pack(pop)

#pragma pack(push, 1)
	typedef struct {
		uint32_t frame_start;

		uint32_t device_id;
		uint16_t is_running;
		uint16_t init_mode_enabled;

		uint32_t frame_end;
	} ManipulatorStatePacket;
#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif // __MANIPULATOR_PACKETS_H__