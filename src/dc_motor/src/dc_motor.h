#ifndef __MOTOR_INPUT_H__
#define __MOTOR_INPUT_H__

#include "globals.h"
#include "gpio.h"
#include "lcd.h"

void Motor_Process(int input);

void Motor_Control(void);

void Motor_DisplayStopped(void);

void Motor_DisplayRunning(void);

#endif
