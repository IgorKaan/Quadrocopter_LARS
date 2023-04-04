#pragma once 

#define STM_START 14

//#define CALIBRATION

// #define RX1_PIN 39 // D1 X9 GREEN TX
// #define TX1_PIN 36 // D2 X10 WHITE RX

#define RX1_PIN 16 // SERV6 X7 GREEN TX
#define TX1_PIN 17 // SERV7 X15 WHITE RX

#define LED1 12
#define LED2 14

#define STABILIZE_MODE

//#define ALTITUDE_HOLD_MODE

#define TARGET_ANGLE 15
//#define TARGET_ANGLE 15

#define FILTER_TASK_HZ          1000/100   // 1000Hz
#define IBUS_READ_TASK_HZ       1000/100    // 100Hz
#define PID_REGULATOR_TASK_HZ   1000/100    // 1000Hz
#define IBUS_LOOP_TASK_HZ       1000/100    // 100 Hz
#define MOTOR_CONTROL_TASK_HZ   1000/1000   // 1000 Hz
#define CAN_RECEIVE_TASK_HZ     1000/1000   // 1000 Hz
#define TFMINI_RECEIVE_TASK_HZ  1000/100    // 100 Hz
#define UART_RECEIVE_TASK_HZ    1000/500    // 200 Hz

#define MIN_JOY_OUTPUT 1100
#define MAX_JOY_OUTPUT 1900

#define MIN_POWER 3400
#define MAX_POWER 6000

#define POWER_INC 1

#define INTEGRAL_COEFFICIENT 0.2

#define PID_OUTPUT 3000

#define PID_I_MAX 40
#define PID_I_MIN -40

// #define PWM_MOTOR_1 25  // D6 X14 LB
// #define PWM_MOTOR_2 32  // D4 X12 RF
// #define PWM_MOTOR_3 33  // D5 X13 RB
// #define PWM_MOTOR_4 26  // D3 X8  LF

#define PWM_MOTOR_1 25  // 25 PWM1 X3 LB
#define PWM_MOTOR_2 32  // 32 PWM3 X5 LF
#define PWM_MOTOR_3 13  // 27 PWM5 X7 RF   13  
#define PWM_MOTOR_4 26  // 26 PWM6 X8 RB

#define PWM_CHANNEL_MOTOR_1  0
#define PWM_CHANNEL_MOTOR_2  1
#define PWM_CHANNEL_MOTOR_3  2
#define PWM_CHANNEL_MOTOR_4  3

#define PWM_FREQUENCY 50

#define PWM_RESOLUTION 16