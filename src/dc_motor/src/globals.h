#ifndef __GLOBALS_H__
#define __GLOBALS_H__

#include <stdint.h>

extern volatile uint32_t    pwm_counter;
extern volatile uint8_t     pwm_bit;

extern volatile uint32_t    motor_speed;
extern volatile uint32_t    motor_direction;

extern volatile uint32_t    timer_counter;

extern volatile uint32_t    keyboard_counter;

#endif
