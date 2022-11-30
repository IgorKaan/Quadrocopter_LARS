#include <Arduino.h>
#include "Fly_sky_iBus.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED1 12
#define LED2 14

#define PWM_MOTOR_1 25  // D6 X14
#define PWM_MOTOR_2 32  // D4 X12
#define PWM_MOTOR_3 33  // D5 X13
#define PWM_MOTOR_4 26  // D3 X8

#define TIMER_NUMBER 0
// Частота ESP32 = 80МГц = 80.000.000Гц, при
// значении 80 частота счётчика 1.000.000
#define TIMER_DIVIDER 80
// Количество тиков счётчика для вызова прерываний.
// При частоте 1МГц, 100000 тиков = 100мс
// ULL - unsigned long long для литералов
#define TIMER_COUNTER 100000ULL
// Указатель на используемый таймер
hw_timer_t *timer = NULL;

uint16_t chanel_read[6];

const int frequency = 50;
   
const int pwmChannelMotor_1 = 0;
const int pwmChannelMotor_2 = 1;
const int pwmChannelMotor_3 = 2;
const int pwmChannelMotor_4 = 3;
    
const int resolution = 11;

long impulse_time[4];

void iBus_read();

void timer_interrupt() {
    
}

void iBusReadTask(void* pvParameters);
void iBusLoopTask(void* pvParameters);
void motorsControlTask(void* pvParameters);

volatile int count = 0;

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

SemaphoreHandle_t serial_mutex;

void iBusReadTask(void* pvParameters){

  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 500 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    xSemaphoreTake(serial_mutex, portMAX_DELAY);
    static ulong time = millis();
    if (millis() - time > 200U) {

    }
    //Serial.print("Task1 running on core ");
    //Serial.println(xPortGetCoreID());
    //count++;
    //Serial.print("Count = ");
    //Serial.println(count);
    iBus_read();
    xSemaphoreGive(serial_mutex);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  } 
}
 

void iBusLoopTask(void* pvParameters){

  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 500 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    xSemaphoreTake(serial_mutex, portMAX_DELAY);
    //Serial.print("Task2 running on core ");
    //Serial.println(xPortGetCoreID());
    //count++;
    iBus.loop();
    //Serial.print("Count = ");
    //Serial.println(count);
    xSemaphoreGive(serial_mutex);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void motorsControlTask(void* pvParameters) {
  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 10 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    // xSemaphoreTake(serial_mutex, portMAX_DELAY);
    if ((chanel_read[4] > 1900) && (chanel_read[5] > 1900)) {
      ledcWrite(pwmChannelMotor_1, map(chanel_read[2], 1000, 2000, 102, 204));
      ledcWrite(pwmChannelMotor_2, map(chanel_read[2], 1000, 2000, 102, 204));
      ledcWrite(pwmChannelMotor_3, map(chanel_read[2], 1000, 2000, 102, 204));
      ledcWrite(pwmChannelMotor_4, map(chanel_read[2], 1000, 2000, 102, 204));
    }
    else {
      ledcWrite(pwmChannelMotor_1, 102);
      ledcWrite(pwmChannelMotor_2, 102);
      ledcWrite(pwmChannelMotor_3, 102);
      ledcWrite(pwmChannelMotor_4, 102);
    }
    // xSemaphoreGive(serial_mutex);
    //Serial.print("Motor run ");
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  } 
}

void setup() {
  Serial.begin(115200);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  pinMode(PWM_MOTOR_3, OUTPUT);
  pinMode(PWM_MOTOR_4, OUTPUT);

  ledcSetup(pwmChannelMotor_1, frequency, resolution);
  ledcSetup(pwmChannelMotor_2, frequency, resolution);
  ledcSetup(pwmChannelMotor_3, frequency, resolution);
  ledcSetup(pwmChannelMotor_4, frequency, resolution);
  ledcAttachPin(PWM_MOTOR_1, pwmChannelMotor_1);
  ledcAttachPin(PWM_MOTOR_2, pwmChannelMotor_2);
  ledcAttachPin(PWM_MOTOR_3, pwmChannelMotor_3);
  ledcAttachPin(PWM_MOTOR_4, pwmChannelMotor_4);

  ledcWrite(pwmChannelMotor_1, 102);
  ledcWrite(pwmChannelMotor_2, 102);
  ledcWrite(pwmChannelMotor_3, 102);
  ledcWrite(pwmChannelMotor_4, 102);

  delay(2500);

  serial_mutex = xSemaphoreCreateMutex();

  BaseType_t t1 = xTaskCreatePinnedToCore(iBusReadTask, "Task1", 5000, NULL, 1, &Task1, 0);
  BaseType_t t2 = xTaskCreatePinnedToCore(iBusLoopTask, "Task2", 5000, NULL, 1, &Task2, 1);
  BaseType_t t3 = xTaskCreatePinnedToCore(motorsControlTask, "Task3", 5000, NULL, 1, &Task3, 1);

  iBus.begin(Serial2);

  delay(1000);

  /// --- Настройка таймера ---
  // Настраиваем 0-й таймер с делителем в 80 тактов
  timer = timerBegin(TIMER_NUMBER, TIMER_DIVIDER, true);
  // Привязываем прерывание к функции timer_interrupt()
  timerAttachInterrupt(timer, &timer_interrupt, true);
  // Настраиваем прерывание на 750мс
  timerAlarmWrite(timer, TIMER_COUNTER, true);
  // Запуск таймера
  timerAlarmEnable(timer);
  /// --- - ---
}

void loop() {
  //Serial.print("loop() running on core ");
 
  //Serial.println(xPortGetCoreID());
  // iBus_read_fun();
  //delay(500);
}

void iBus_read()
{
  static uint16_t chanel[6] = {0, 0, 0, 0, 0, 0};
  // uint16_t chanel_read[6];
  // iBus.loop();

  bool signal = false;
  for (int i = 0; i < 6; ++i)
  {
    chanel_read[i] = iBus.readChannel(i);

    if ((chanel[i] <= (chanel_read[i] - 5)) || ((chanel[i] >= (chanel_read[i] + 5))))
      signal = true;
  }
  Serial.print("Channel 1 = ");
  Serial.println(chanel_read[0], DEC);
  Serial.print("Channel 2 = ");
  Serial.println(chanel_read[1], DEC);
  Serial.print("Channel 3 = ");
  Serial.println(chanel_read[2], DEC);
  Serial.print("Channel 4 = ");
  Serial.println(chanel_read[3], DEC);
  Serial.print("Channel 5 = ");
  Serial.println(chanel_read[4], DEC);
  Serial.print("Channel 6 = ");
  Serial.println(chanel_read[5], DEC);
  Serial.println("==================================================");
  //delay(500);
}