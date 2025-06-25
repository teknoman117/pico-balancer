#include "motors.hpp"

#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/pwm.h>

#include "quadrature_encoder.pio.h"

#define MOTOR_LEFT_PWM 16
#define MOTOR_LEFT_A 18
#define MOTOR_LEFT_B 17

#define MOTOR_RIGHT_PWM 21
#define MOTOR_RIGHT_A 20
#define MOTOR_RIGHT_B 19

#define MOTOR_PWM_WRAP 5000
#define MOTOR_DEADZONE 50

#define MOTOR_LEFT_ENCODER_A 14
#define MOTOR_LEFT_ENCODER_B 15
#define MOTOR_RIGHT_ENCODER_A 12
#define MOTOR_RIGHT_ENCODER_B 13

namespace {
    int32_t previous_left_encoder = 0;
    int32_t previous_right_encoder = 0;
}

void setup_motors() {
    // set up the motor GPIOs and PWM
    gpio_init(MOTOR_LEFT_A);
    gpio_set_dir(MOTOR_LEFT_A, GPIO_OUT);
    gpio_put(MOTOR_LEFT_A, false);

    gpio_init(MOTOR_LEFT_B);
    gpio_set_dir(MOTOR_LEFT_B, GPIO_OUT);
    gpio_put(MOTOR_LEFT_B, false);

    gpio_init(MOTOR_RIGHT_A);
    gpio_set_dir(MOTOR_RIGHT_A, GPIO_OUT);
    gpio_put(MOTOR_RIGHT_A, false);

    gpio_init(MOTOR_RIGHT_B);
    gpio_set_dir(MOTOR_RIGHT_B, GPIO_OUT);
    gpio_put(MOTOR_RIGHT_B, false);

    gpio_set_function(MOTOR_LEFT_PWM, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_gpio_to_slice_num(MOTOR_LEFT_PWM), MOTOR_PWM_WRAP);
    pwm_set_enabled(pwm_gpio_to_slice_num(MOTOR_LEFT_PWM), true);

    gpio_set_function(MOTOR_RIGHT_PWM, GPIO_FUNC_PWM);
    pwm_set_wrap(pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM), MOTOR_PWM_WRAP);
    pwm_set_enabled(pwm_gpio_to_slice_num(MOTOR_RIGHT_PWM), true);
}

void setup_encoders() {
    pio_add_program(pio0, &quadrature_encoder_program);

    quadrature_encoder_program_init(pio0, 0, MOTOR_LEFT_ENCODER_A, 0);
    quadrature_encoder_program_init(pio0, 1, MOTOR_RIGHT_ENCODER_A, 0);
}

void set_motor_left_speed(float speed_) {
    auto speed = (speed_ > 1.f ? 1.f : speed_ < -1.f ? -1.f : speed_)
            * (float) (MOTOR_PWM_WRAP - MOTOR_DEADZONE);

    if (speed > 0) {
        gpio_put(MOTOR_LEFT_A, false);
        gpio_put(MOTOR_LEFT_B, true);
        pwm_set_gpio_level(MOTOR_LEFT_PWM, speed + MOTOR_DEADZONE);
    } else if (speed < 0) {
        gpio_put(MOTOR_LEFT_A, true);
        gpio_put(MOTOR_LEFT_B, false);
        pwm_set_gpio_level(MOTOR_LEFT_PWM, -speed - MOTOR_DEADZONE);
    } else {
        gpio_put(MOTOR_LEFT_A, false);
        gpio_put(MOTOR_LEFT_B, false);
    }
}

void set_motor_right_speed(float speed_) {
    auto speed = (speed_ > 1.f ? 1.f : speed_ < -1.f ? -1.f : speed_)
            * (float) (MOTOR_PWM_WRAP - MOTOR_DEADZONE);

    if (speed > 0) {
        gpio_put(MOTOR_RIGHT_A, true);
        gpio_put(MOTOR_RIGHT_B, false);
        pwm_set_gpio_level(MOTOR_RIGHT_PWM, speed + MOTOR_DEADZONE);
    } else if (speed < 0) {
        gpio_put(MOTOR_RIGHT_A, false);
        gpio_put(MOTOR_RIGHT_B, true);
        pwm_set_gpio_level(MOTOR_RIGHT_PWM, -speed - MOTOR_DEADZONE);
    } else {
        gpio_put(MOTOR_RIGHT_A, false);
        gpio_put(MOTOR_RIGHT_B, false);
    }
}

int32_t get_motor_velocity() {
    int32_t left_encoder = quadrature_encoder_get_count_nonblocking(pio0, 0, previous_left_encoder);
    int32_t right_encoder = quadrature_encoder_get_count_nonblocking(pio0, 1, previous_right_encoder);
    int32_t left_encoder_delta = left_encoder - previous_left_encoder;
    int32_t right_encoder_delta = right_encoder - previous_right_encoder;
    previous_left_encoder = left_encoder;
    previous_right_encoder = right_encoder;
    return (left_encoder_delta + right_encoder_delta) / 2;
}