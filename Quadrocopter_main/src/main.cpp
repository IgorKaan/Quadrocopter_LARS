#include <Arduino.h>
#include "thread.hpp"
#include "config.hpp"
#include "Fly_sky_iBus.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ESP32CAN.h>

void timer_interrupt();
void iBusReadTask(void* pvParameters);
void iBusLoopTask(void* pvParameters);
void canReceiveTask(void* pvParameters);
void pidRegulatorTask(void* pvParameters);
void motorsControlTask(void* pvParameters);

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

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t Task5;

SemaphoreHandle_t serial_mutex;
SemaphoreHandle_t param_mutex;

// --- CAN шина ---
CAN_device_t CAN_cfg;             // Конфигурация CAN
unsigned long previousMillis = 0; // Время последней отправки CAN сообщения
const int interval = 500;         // Интервал отправки CAN сообщений (мс)
const int rx_queue_size = 1;     // Размер буфера приёма
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_23;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_19;

void setup() {
  Serial.begin(115200);

  // Светодиоды
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  // Светодиоды

  // Инициализация двигателей
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
  // Инициализация двигателей

  delay(2500);

  // Создание мьютексов
  param_mutex = xSemaphoreCreateMutex();
  serial_mutex = xSemaphoreCreateMutex();
  // Создание мьютексов

  // Инициализация приемника
  iBus.begin(Serial2);
  // Инициализация приемника

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
  /// --- Настройка таймера ---

  // Инициализация CAN шины
  CAN_cfg.tx_pin_id = CAN_TX_PIN;
  CAN_cfg.rx_pin_id = CAN_RX_PIN;
  CAN_cfg.speed = CAN_SPEED_250KBPS;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  ESP32Can.CANInit();
  // Инициализация CAN шины

  // Создание тасков
  BaseType_t t1 = xTaskCreatePinnedToCore(iBusReadTask, "Task1", 5000, NULL, 1, &Task1, 0);
  BaseType_t t2 = xTaskCreatePinnedToCore(iBusLoopTask, "Task2", 5000, NULL, 1, &Task2, 1);
  BaseType_t t3 = xTaskCreatePinnedToCore(motorsControlTask, "Task3", 5000, NULL, 1, &Task3, 1);
  BaseType_t t4 = xTaskCreatePinnedToCore(canReceiveTask, "Task4", 30000, NULL, 1, &Task4, 1);
  BaseType_t t5 = xTaskCreatePinnedToCore(pidRegulatorTask, "Task5", 5000, NULL, 1, &Task5, 0);
  // Создание тасков
}
extern float yaw, pitch, roll;
void loop() {
    Serial.print("roll: ");
    Serial.print(roll);
    Serial.print("\t\t");
    Serial.print("pitch: ");
    Serial.print(pitch);
    Serial.print("\t\t");
    Serial.print("yaw: ");
    Serial.println(yaw);
    Serial.println("===============================");
    delay(100);
}
