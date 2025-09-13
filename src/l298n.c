#include "l298n.h"

#include "hardware/gpio.h"

void l298n_init(l298n* motor)
{
    gpio_init(motor->pwml_pin);
    gpio_set_dir(motor->pwml_pin, GPIO_OUT);
    gpio_put(motor->pwml_pin, 0);

    gpio_init(motor->pwmr_pin);
    gpio_set_dir(motor->pwmr_pin, GPIO_OUT);
    gpio_put(motor->pwmr_pin, 0); 
}

void l298n_speed_set(l298n* motor, float speed)
{
    if (speed > 0.0f) {
        gpio_put(motor->pwml_pin, 1);
        gpio_put(motor->pwmr_pin, 0);
        motor->pwm_duty = 100.0f;
    } else if (speed < 0.0f) {
        gpio_put(motor->pwml_pin, 0);
        gpio_put(motor->pwmr_pin, 1);
        motor->pwm_duty = -100.0f;
    } else {
        gpio_put(motor->pwml_pin, 0);
        gpio_put(motor->pwmr_pin, 0);
        motor->pwm_duty = 0.0f;
    }
}