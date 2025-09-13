/*
 * controller_params.h
 *
 *  Created on: Sep 4, 2025
 *      Author: alper
 */

#ifndef INC_CONTROLLER_PARAMS_H_
#define INC_CONTROLLER_PARAMS_H_
 
#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float p_bound;
    float i_bound;
    float d_bound;

    float i;
    float prev_err;

    float delta_t;
} pid_params;

typedef struct {
    float kv;
    float ka;
    float kj;

    float v_offset;
} feedforward_params;

void pid_params_init(pid_params* pid,
    float kp, float ki, float kd,
    float p_bound, float i_bound, float d_bound, float fs);
 
float pid_params_update(pid_params* pid, float err);
 
float pid_params_update_d_filtered(pid_params* pid, float err, float d_filtered);
 
void feedforward_params_init(feedforward_params* ff,
    float kv, float ka, float kj, float v_offset);
 
float feedforward_params_update(feedforward_params* ff,
    float vel_ref, float acc_ref, float j_ref);
 
 #endif