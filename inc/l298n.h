/*
 * l298n.h
 *
 *  Created on: Sep 13, 2025
 *      Author: alper
 */

#ifndef INC_L298N_H_
#define INC_L298N_H_

#include <stdint.h>

typedef struct {
    uint32_t pwml_pin;
    uint32_t pwmr_pin;

    float pwm_duty;

} l298n;

void l298n_init(l298n* motor);

void l298n_speed_set(l298n* motor, float speed);

#endif