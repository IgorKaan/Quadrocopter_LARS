#include "PID.hpp"
#include "TFMPlus.h"
#include <Arduino.h>
#include "thread.hpp"
#include "config.hpp"
#include "Fly_sky_iBus.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ESP32CAN.h>
#include <TroykaIMU.h>

// множитель фильтра
#define BETA 0.22f
 
// создаём объект для фильтра Madgwick
Madgwick filter;

void timer_interrupt();
void iBusReadTask(void* pvParameters);
void iBusLoopTask(void* pvParameters);
void UARTReadTask(void* pvParameters);
void canReceiveTask(void* pvParameters);
void TFMiniReadTask(void* pvParameters);
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
TaskHandle_t Task6;
TaskHandle_t Task7;

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
  #ifndef CALIBRATION
  ledcWrite(PWM_CHANNEL_MOTOR_1, MIN_POWER);
  ledcWrite(PWM_CHANNEL_MOTOR_2, MIN_POWER);
  ledcWrite(PWM_CHANNEL_MOTOR_3, MIN_POWER);
  ledcWrite(PWM_CHANNEL_MOTOR_4, MIN_POWER);
  #endif
  #ifdef CALIBRATION
  ledcWrite(PWM_CHANNEL_MOTOR_1, MAX_POWER);
  ledcWrite(PWM_CHANNEL_MOTOR_2, MAX_POWER);
  ledcWrite(PWM_CHANNEL_MOTOR_3, MAX_POWER);
  ledcWrite(PWM_CHANNEL_MOTOR_4, MAX_POWER);
  delay(5000);
  for (int i = MAX_POWER; i > MIN_POWER; --i) {
  // ledcWrite(PWM_CHANNEL_MOTOR_1, i);
  // ledcWrite(PWM_CHANNEL_MOTOR_2, i);
  // ledcWrite(PWM_CHANNEL_MOTOR_3, i);
  // ledcWrite(PWM_CHANNEL_MOTOR_4, i);
  // delay(1);
  }
  delay(2500);
  #endif
  // Инициализация двигателей
  delay(5000);

  // Создание мьютексов
  param_mutex = xSemaphoreCreateMutex();
  serial_mutex = xSemaphoreCreateMutex();
  // Создание мьютексов

  Serial.begin(115200);

  // Инициализация приемника
  iBus.begin(Serial2);
  // Инициализация приемника

  delay(1000);

  Serial1.begin(921600, SERIAL_8N1, RX1_PIN, TX1_PIN);
  Serial.println("start");
  // Запуск СТМ
  pinMode(STM_START, OUTPUT); // STM_START
  delay(250);
  digitalWrite(STM_START, 1);
  // Запуск СТМ 
  // tfmP.begin(&Serial1);

  // if(tfmP.sendCommand(SOFT_RESET, 0)) {
  //   Serial.println("TFMini soft reset... ");
  // }
  // //delay(1000);
  // if (tfmP.sendCommand(GET_FIRMWARE_VERSION, 0)) {
  //   Serial.println("TFMini version: ");
  //   Serial.print(tfmP.version[0]); // print three single numbers
  //   Serial.print(".");
  //   Serial.print(tfmP.version[1]); // each separated by a dot
  //   Serial.print(".");
  //   Serial.println(tfmP.version[2]);
  // }
  // else {
  //   Serial.println("Failed to initialize TFMini");
  // }
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
  // CAN_cfg.tx_pin_id = CAN_TX_PIN;
  // CAN_cfg.rx_pin_id = CAN_RX_PIN;
  // CAN_cfg.speed = CAN_SPEED_250KBPS;
  // CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // ESP32Can.CANInit();
  // Инициализация CAN шины

  // Создание тасков
  BaseType_t t1 = xTaskCreatePinnedToCore(iBusReadTask, "Task1", 5000, NULL, 1, &Task1, 1);
  BaseType_t t2 = xTaskCreatePinnedToCore(iBusLoopTask, "Task2", 5000, NULL, 1, &Task2, 1);
  BaseType_t t3 = xTaskCreatePinnedToCore(motorsControlTask, "Task3", 5000, NULL, 1, &Task3, 1);
  BaseType_t t4 = xTaskCreatePinnedToCore(canReceiveTask, "Task4", 30000, NULL, 1, &Task4, 1);
  BaseType_t t5 = xTaskCreatePinnedToCore(pidRegulatorTask, "Task5", 5000, NULL, 1, &Task5, 1);
  BaseType_t t6 = xTaskCreatePinnedToCore(TFMiniReadTask, "Task6", 5000, NULL, 1, &Task6, 0);
  BaseType_t t7 = xTaskCreatePinnedToCore(UARTReadTask, "Task7", 30000, NULL, 1, &Task7, 0);
  // Создание тасков

}

extern PIDImpl pidRoll;
extern PIDImpl pidPitch;
extern float deg_pitch, deg_roll;
extern float yaw, pitch, roll;
extern float altitude, powerLB;
extern float targetRoll, targetPitch, targetYaw;
extern float targetPowerRF, targetPowerRB, targetPowerLF, targetPowerLB;
extern float errorRoll, errorPitch, errorYaw;
extern float additionalPowerRF, additionalPowerRB, additionalPowerLF, additionalPowerLB;
extern float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;

void loop() {
    // Serial.println("loop");
    // //if (Serial1.available()) {
    //   Serial.println("receive");
    //   uint8_t sByte = Serial1.read();
    
    // if (sByte == 83) {
    //   Serial.println("receive");
    //   Serial.println(sByte);
    //   Serial1.readBytes(buffer, 24);
    //   memcpy(&f1, &buffer[0], sizeof(float));
    //   memcpy(&f2, &buffer[4], sizeof(float));
    //   memcpy(&f3, &buffer[8], sizeof(float));
    //   memcpy(&f4, &buffer[12], sizeof(float));
    //   memcpy(&f5, &buffer[16], sizeof(float));
    //   memcpy(&f6, &buffer[20], sizeof(float));
    //   delay(500);
    //   Serial.println(f1);
    //   Serial.println(f2);
    //   Serial.println(f3);
    //   Serial.println(f4);
    //   Serial.println(f5);
    //   Serial.println(f6);
    //   Serial.println("----------------");
    // }
  //#ifdef DEBUG_PRINT
    Serial.print("roll: ");
    Serial.print(roll, 5);
    Serial.print("\t\t");
    Serial.print("pitch: ");
    Serial.print(pitch, 5);
    Serial.print("\t\t");
    Serial.print("yaw: ");
    Serial.println(yaw, 5);
    Serial.println("===============================");
    // // delay(1000);
    Serial.print("error roll: ");
    Serial.print(errorRoll);
    Serial.print("\t");
    Serial.print("error pitch: ");
    Serial.print(errorPitch);
    Serial.print("\t");
    Serial.print("error yaw: ");
    Serial.println(errorYaw);
    Serial.println("===============================");
    // Serial.print("P koeff: ");
    // Serial.print(powerLB);
    // Serial.print("\t\t");
    // Serial.print("D koeff: ");
    // Serial.println(pidRoll.getDcoefficient());
    // Serial.println("===============================");
    // Serial.println(deg_roll, 3);
    // Serial.println(deg_pitch, 3);
    // Serial.println(yaw, 3);
    // if (deg_roll > 5) {
    //   Serial.println("boom");
    // }
    // Serial.println(accelX, 5);
    // Serial.println(accelY, 5);
    // Serial.println(accelZ, 5);
    //Serial.println("----------------");

    //filter.setKoeff(10, BETA);
    // обновляем входные данные в фильтр
    //filter.update(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
  
    // получение углов yaw, pitch и roll из фильтра
    // yaw =  filter.getYawDeg();
    // pitch = filter.getPitchDeg();
    // roll = filter.getRollDeg();

    // Serial.println(roll, 5);
    // Serial.println(pitch, 5);
    // Serial.println(yaw, 5);
    // Serial.println("----------------");
    //delay(100);
  //#endif
  delay(100);
}
