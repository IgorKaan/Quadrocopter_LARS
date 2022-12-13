#include <Arduino.h>
#include <esp32-hal-ledc.h>
#define LED1 12
#define LED2 14

#define MIN_POWER 3277
#define MAX_POWER 6544

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


void setup() {
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  pinMode(PWM_MOTOR_3, OUTPUT);
  pinMode(PWM_MOTOR_4, OUTPUT);
  ledcSetup(PWM_CHANNEL_MOTOR_1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_MOTOR_2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_MOTOR_3, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_MOTOR_4, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(PWM_MOTOR_1, PWM_CHANNEL_MOTOR_1);
  ledcAttachPin(PWM_MOTOR_2, PWM_CHANNEL_MOTOR_2);
  ledcAttachPin(PWM_MOTOR_3, PWM_CHANNEL_MOTOR_3);
  ledcAttachPin(PWM_MOTOR_4, PWM_CHANNEL_MOTOR_4);
  ledcWrite(PWM_CHANNEL_MOTOR_1, MIN_POWER);
  ledcWrite(PWM_CHANNEL_MOTOR_2, MIN_POWER);
  ledcWrite(PWM_CHANNEL_MOTOR_3, MIN_POWER);
  ledcWrite(PWM_CHANNEL_MOTOR_4, MIN_POWER);
  delay(2500);
}

void loop() {
  ledcWrite(PWM_CHANNEL_MOTOR_1, 3500);
  ledcWrite(PWM_CHANNEL_MOTOR_2, 3500);
  ledcWrite(PWM_CHANNEL_MOTOR_3, 3500);
  ledcWrite(PWM_CHANNEL_MOTOR_4, 3500);
  //ledcWrite(pwmChannel, 4000);
  // digitalWrite(LED1, HIGH);
  // delay(3000);
  
  // digitalWrite(LED1, LOW);
  // delay(3000);
}