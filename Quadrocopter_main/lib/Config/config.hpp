#pragma once 

//#define CALIBRATION

#define LED1 12
#define LED2 14

#define TARGET_ANGLE 30
//#define TARGET_ANGLE 15

#define MIN_JOY_OUTPUT 1100
#define MAX_JOY_OUTPUT 1900

#define MIN_POWER 3276
#define MAX_POWER 6000

#define POWER_INC 1

#define INTEGRAL_COEFFICIENT 0.5

#define PID_OUTPUT 2000

#define PID_I_MAX 200
#define PID_I_MIN -200

#define PWM_MOTOR_1 25  // D6 X14 LB
#define PWM_MOTOR_2 32  // D4 X12 RF
#define PWM_MOTOR_3 33  // D5 X13 RB
#define PWM_MOTOR_4 26  // D3 X8  LF

#define PWM_CHANNEL_MOTOR_1  0
#define PWM_CHANNEL_MOTOR_2  1
#define PWM_CHANNEL_MOTOR_3  2
#define PWM_CHANNEL_MOTOR_4  3

#define PWM_FREQUENCY 50

#define PWM_RESOLUTION 16