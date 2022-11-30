#include <Arduino.h>
#include <esp32-hal-ledc.h>
#define LED1 12
#define LED2 14

#define PWM_MOTOR_1 26
#define PWM_MOTOR_2 27

const int frequency = 50;
   
const int pwmChannel = 0;
    
const int resolution = 16;


void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  ledcSetup(pwmChannel, frequency, resolution);
  ledcAttachPin(PWM_MOTOR_1, pwmChannel);
  //ledcWrite(pwmChannel, 0);
  //delay(5000);
  //ledcWrite(pwmChannel, 100);
  //delay(5000);
  ledcWrite(pwmChannel, 3277);
  delay(2500);
}

void loop() {
  delay(3000);
  int i = 3277;
  while (i < 6554) {
    ledcWrite(pwmChannel, i);
    ++i;
    delay(10);
  }
  ledcWrite(pwmChannel, 3277);
  // digitalWrite(LED1, HIGH);
  // delay(3000);
  
  // digitalWrite(LED1, LOW);
  // delay(3000);
}