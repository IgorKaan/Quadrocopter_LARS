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
#include "Biquad.h"
#include <string.h>
#include <string>
#include <Chebishev.h>
#include <GoodMorningCopter.h>
unsigned long timing;
// множитель фильтра
#define BETA 0.22f
extern PIDImpl pidRoll;
extern PIDImpl pidPitch;
extern float deg_pitch, deg_roll;
extern float yaw, pitch, roll, TrueYaw, SumErrorsYaw, OldSumErrorsYaw, ErrorYaw, SumRiseErrorYaw, RiseErrorYaw;
extern float altitude, powerLB, powerLF, powerRB, powerRF;
extern float targetRoll, targetPitch, targetYaw;
extern float targetPowerRF, targetPowerRB, targetPowerLF, targetPowerLB;
extern float errorRoll, errorPitch, errorYaw;
extern float additionalPowerRF, additionalPowerRB, additionalPowerLF, additionalPowerLB;
extern float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
extern float BiquadNotchAccelX, BiquadNotchAccelY, BiquadNotchAccelZ, BiquadNotchGyroX, BiquadNotchGyroY, BiquadNotchGyroZ,  BiquadLowPassGyroX,  BiquadLowPassGyroY,  BiquadLowPassGyroZ, BiquadLowPassAccelX, BiquadLowPassAccelY, BiquadLowPassAccelZ;;
extern float f1, f2, f3, f4, f5, f6; 
extern float SumErrorsGyroX, SumErrorsGyroY, SumErrorsGyroZ, ErrorGyroX, ErrorGyroY, ErrorGyroZ;
extern float SumErrorsAccelX, SumErrorsAccelY, SumErrorsAccelZ, ErrorAccelX, ErrorAccelY, ErrorAccelZ;
extern float ChebishevGyroX, ChebishevGyroY, ChebishevGyroZ, ChebishevAccelX, ChebishevAccelY, ChebishevAccelZ;
extern float LowPassGyroX, LowPassGyroY, LowPassGyroZ, LowPassAccelX, LowPassAccelY, LowPassAccelZ;
extern float PT1GyroX, PT1GyroY, PT1GyroZ, PT1AccelX, PT1AccelY, PT1AccelZ;
float PercentPowerRF, PercentPowerRB, PercentPowerLF, PercentPowerLB, PowerToPercent;
// создаём объект для фильтра Madgwick
Madgwick filter;

void timer_interrupt();
void filterTask(void* pvParameters);
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
TaskHandle_t Task8;

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
  WakeUp();
 // ReadyToCalibration();
  // Светодиоды
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  // Светодиоды
  //float x1 = Serial2.read();
 // float x2 = Serial2.read();
 // float x3 = Serial2.read();
// float x4 = Serial2.read();
//  float x5 = Serial2.read();
//  float x6 = Serial2.read();
 //ReadyToCalibration();
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

  BaseType_t t7 = xTaskCreatePinnedToCore(UARTReadTask, "Task7", 30000, NULL, 1, &Task7, 0);
  delay(3000);
  ReadyToCalibration();
  int i = 0;
  for( i; i < 1000; i++)
  //while(millis() < 10000)
  {
  SumErrorsGyroX+=/*f4*/ BiquadNotchGyroX - 0;
  SumErrorsGyroY+=/*f5*/ BiquadNotchGyroY - 0;
  SumErrorsGyroZ+=/*f6*/ BiquadNotchGyroZ - 0;
  SumErrorsAccelX += /*f1*/ BiquadNotchAccelX - 0;
  SumErrorsAccelY += /*f2*/ BiquadNotchAccelY - 0;
  SumErrorsAccelZ += /*f3*/ BiquadNotchAccelZ + 9.81;
  //i++;
  delay(5);
  }
  ErrorGyroX = SumErrorsGyroX / i;
  ErrorGyroY = SumErrorsGyroY / i;
  ErrorGyroZ = SumErrorsGyroZ / i;
  ErrorAccelX = SumErrorsAccelX / i;
  ErrorAccelY = SumErrorsAccelY / i;
  ErrorAccelZ = SumErrorsAccelZ / i;
  //PowerToPercent = (MAX_POWER - MIN_POWER) / 100;
  //Serial.println(x);
  //Serial.println(SumRiseErrorYaw);
  //Serial.println(RiseErrorYaw);
  EndCalibration();

  // Создание тасков
  BaseType_t t1 = xTaskCreatePinnedToCore(iBusReadTask, "Task1", 5000, NULL, 1, &Task1, 1);
  BaseType_t t2 = xTaskCreatePinnedToCore(iBusLoopTask, "Task2", 5000, NULL, 1, &Task2, 1);
  BaseType_t t3 = xTaskCreatePinnedToCore(motorsControlTask, "Task3", 5000, NULL, 1, &Task3, 1);
  BaseType_t t4 = xTaskCreatePinnedToCore(canReceiveTask, "Task4", 30000, NULL, 1, &Task4, 1);
  BaseType_t t5 = xTaskCreatePinnedToCore(pidRegulatorTask, "Task5", 5000, NULL, 1, &Task5, 1);
  BaseType_t t6 = xTaskCreatePinnedToCore(TFMiniReadTask, "Task6", 5000, NULL, 1, &Task6, 0);
  
  BaseType_t t8 = xTaskCreatePinnedToCore(filterTask, "Task8", 30000, NULL, 1, &Task8, 0);
  // Создание тасков
}






void loop() {

 // Serial.println("Time from the start of the program: " + (String)millis() + " (ms). PWM signal:");
 // Serial.println("Rotor-1: " + (String)powerLF + "       Rotor-2: " + (String)powerRF + "       Rotor-3: " + (String)powerLB + "       Rotor-4: " + (String)powerRB + "\n");
//  Serial.println("Time from the start of the program: " + (String)millis() + " (ms). Gyroscope data (XYZ):");
//  Serial.println("X-axis: " + (String)f4 + "       Y-axis: " + (String)f5 + "       Z-axis: "+ (String)f6 + "\n");
//  Serial.println("Time from the start of the program: " + (String)millis() + " (ms). Accelerometer data (XYZ):");
 // Serial.println("X-axis: " + (String)f1 + "       Y-axis: " + (String)f2 + "       Z-axis: "+ (String)f3 + "\n");
 // Serial.println("Time from the start of the program: " + (String)millis() + " (ms). Filtered gyroscope data by low-pass filter (XYZ):");
//  Serial.println("X-axis: " + (String)lPTgyroX + "       Y-axis: " + (String)lPTgyroY + "       Z-axis: "+ (String)lPTgyroZ + "\n");
//  Serial.println("Time from the start of the program: " + (String)millis() + " (ms). Filtered accelerometer data by low-pass filter (XYZ):");
//  Serial.println("X-axis: " + (String)lPTaccelX + "       Y-axis: " + (String)lPTaccelY + "       Z-axis: "+ (String)lPTaccelZ + "\n");
//  Serial.println("Time from the start of the program: " + (String)millis() + " (ms). Filtered gyroscope data by low-pass & notch filter (XYZ):");
//  Serial.println("X-axis: " + (String)nPTgyroX + "       Y-axis: " + (String)nPTgyroY + "       Z-axis: "+ (String)nPTgyroZ + "\n");
 // Serial.println("Time from the start of the program: " + (String)millis() + " (ms). Filtered accelerometer data by low-pass & notch filter (XYZ):");
 // Serial.println("X-axis: " + (String)nPTaccelX + "       Y-axis: " + (String)nPTaccelY + "       Z-axis: "+ (String)nPTaccelZ + "\n");
 // Serial.println("Time from the start of the program: " + (String)millis() + " (ms). Obtained angles after filtering by the Madgwick filter (Roll, Pitch, Yaw):");
 // Serial.println("Roll: " + (String)roll + "       Pitch: " + (String)pitch + "       Yaw: "+ (String)yaw + "\n");
 // Serial.println("Time from the start of the program: " + (String)millis() + " (ms). Target Positions (Roll, Pitch, Yaw):");
 // Serial.println("Target Roll: " + (String)targetRoll + "       Target Pitch: " + (String)targetPitch + "       Target Yaw: "+ (String)targetYaw + "\n");
 // Serial.println("Time from the start of the program: " + (String)millis() + " (ms). Position error (Roll, Pitch, Yaw):");
 // Serial.println("Error Roll: " + (String)errorRoll + "       Error Pitch: " + (String)errorPitch + "       Error Yaw: "+ (String)errorYaw);
 // Serial.print("\n\n\n\n\n\n\n");
 // delay(200);
  // Serial.print(" ");
  // Serial.print(" ");
  // Serial.print(accelXRaw);
  // Serial.print(" ");
   
   
   //Serial.print(targetPowerLB); //25
  // Serial.print("\t");
   //Serial.println(targetPowerRF); //25
 //  Serial.println();
 //   Serial.print("\t");
 //  Serial.println(targetPowerRB); //27
 //  Serial.println();
 //  Serial.println();

  
  // Serial.println();
  //#ifdef DEBUG_PRINT
 /*PercentPowerLB = (powerLB - MIN_POWER) / PowerToPercent;
 PercentPowerLF = (powerLF - MIN_POWER) / PowerToPercent;
 PercentPowerRB = (powerRB - MIN_POWER) / PowerToPercent;
 PercentPowerRF = (powerRF - MIN_POWER) / PowerToPercent; */
  if (millis() - timing > 10) { // Вместо 10000 подставьте нужное вам значение паузы 
  timing = millis(); 
  char str[1000];
  sprintf (str, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",  f4, f5, f6, f1, f2, f3, BiquadLowPassGyroX, BiquadLowPassGyroY, BiquadLowPassGyroZ, BiquadLowPassAccelX, BiquadLowPassAccelY, BiquadLowPassAccelZ, BiquadNotchGyroX, BiquadNotchGyroY, BiquadNotchGyroZ, BiquadNotchAccelX, BiquadNotchAccelY, BiquadNotchAccelZ, deg_roll, deg_pitch, yaw, targetPowerLF, targetPowerRF, targetPowerLB, targetPowerRB, ChebishevGyroX, ChebishevGyroY, ChebishevGyroZ, ChebishevAccelX, ChebishevAccelY, ChebishevAccelZ, LowPassGyroX, LowPassGyroY, LowPassGyroZ, LowPassAccelX, LowPassAccelY, LowPassAccelZ, PT1GyroX, PT1GyroY, PT1GyroZ, PT1AccelX, PT1AccelY, PT1AccelZ, targetRoll, targetPitch, targetYaw, errorRoll, errorPitch, errorYaw);
 
  //string k = "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", f4, f5, f6, f1, f2, f3, BiquadLowPassGyroX, BiquadLowPassGyroY, BiquadLowPassGyroZ, BiquadLowPassAccelX, BiquadLowPassAccelY, BiquadLowPassAccelZ, BiquadNotchGyroX, BiquadNotchGyroY, BiquadNotchGyroZ, BiquadNotchAccelX, BiquadNotchAccelY, BiquadNotchAccelZ, deg_roll, deg_pitch, yaw, targetPowerLF, targetPowerRF, targetPowerLB, targetPowerRB, ChebishevGyroX, ChebishevGyroY, ChebishevGyroZ, ChebishevAccelX, ChebishevAccelY, ChebishevAccelZ, LowPassGyroX, LowPassGyroY, LowPassGyroZ, LowPassAccelX, LowPassAccelY, LowPassAccelZ, PT1GyroX, PT1GyroY, PT1GyroZ, PT1AccelX, PT1AccelY, PT1AccelZ, targetRoll, targetPitch, targetYaw, errorRoll, errorPitch, errorYaw;
  Serial.print(str);
   /* Serial.print(" ");
  
    Serial.print(f5);49
    Serial.print(" ");
    Serial.print(f6);48
    Serial.print(" ");
    Serial.print(f1);47
    Serial.print(" ");
    Serial.print(f2);46
    Serial.print(" ");
    Serial.print(f3);
    Serial.print(" "); 45
   Serial.print(BiquadLowPassGyroX);44
   Serial.print(" ");
   Serial.print(BiquadLowPassGyroY);43
   Serial.print(" ");
   Serial.print(BiquadLowPassGyroZ);42
   Serial.print(" ");
   Serial.print(BiquadLowPassAccelX);41
   Serial.print(" ");
   Serial.print(BiquadLowPassAccelY);40
   Serial.print(" ");
   Serial.print(BiquadLowPassAccelZ);39
   Serial.print(" ");
   Serial.print(BiquadNotchGyroX);38
   Serial.print(" ");
   Serial.print(BiquadNotchGyroY);37
   Serial.print(" ");
   Serial.print(BiquadNotchGyroZ);36
   Serial.print(" ");
   Serial.print(BiquadNotchAccelX);35
   Serial.print(" ");
   Serial.print(BiquadNotchAccelY);34
   Serial.print(" ");
   Serial.print(BiquadNotchAccelZ);33
   Serial.print(" "); 
   Serial.print(deg_roll);32
   Serial.print(" ");
   Serial.print(deg_pitch);31
   Serial.print(" ");
   Serial.print(yaw);30
   Serial.print(" ");
   Serial.print(targetPowerLF);29
   Serial.print(" ");
   Serial.print(targetPowerRF);28
   Serial.print(" ");
   Serial.print(targetPowerLB);27
   Serial.print(" ");
   Serial.print(targetPowerRB);26
   Serial.print(" "); 
   Serial.print(ChebishevGyroX);25
   Serial.print(" ");
   Serial.print(ChebishevGyroY);24
   Serial.print(" ");
   Serial.print(ChebishevGyroZ);23
   Serial.print(" ");
   Serial.print(ChebishevAccelX);22
   Serial.print(" ");
   Serial.print(ChebishevAccelY); 21
   Serial.print(" ");
   Serial.print(ChebishevAccelZ); 20
   Serial.print(" ");
   Serial.print(LowPassGyroX);19
   Serial.print(" ");
   Serial.print(LowPassGyroY);18
   Serial.print(" ");
   Serial.print(LowPassGyroZ);17
   Serial.print(" ");
   Serial.print(LowPassAccelX);16
   Serial.print(" ");
   Serial.print(LowPassAccelY); 15
   Serial.print(" ");
   Serial.print(LowPassAccelZ);  14
   Serial.print(" ");
   Serial.print(PT1GyroX);13
   Serial.print(" ");
   Serial.print(PT1GyroY); 12
   Serial.print(" ");
   Serial.print(PT1GyroZ); 11
   Serial.print(" ");
   Serial.print(PT1AccelX); 10
   Serial.print(" ");
   Serial.print(PT1AccelY); 9
   Serial.print(" ");
   Serial.print(PT1AccelZ); 8
   Serial.print(" ");
   Serial.print(targetRoll);7
   Serial.print(" ");
   Serial.print(targetPitch); 6
   Serial.print(" ");
   Serial.print(targetYaw); 5
   Serial.print(" ");
   Serial.print(errorRoll);4
   Serial.print(" ");
   Serial.print(errorPitch); 3
   Serial.print(" ");
   Serial.print(errorYaw); 2
   Serial.print(" ");
   Serial.println(millis()); 8*/ 
  }
   // Serial.print(accelX, 5);
   // Serial.print("\t\t");
   // Serial.print("pitch: ");
   // Serial.print(accelY, 5);
   // Serial.print("\t\t");
   // Serial.print("yaw: ");
  //  Serial.println(accelZ, 5);
  // Serial.println("===============================");
   // Serial.print("error roll: ");
    //Serial.print(f4);
   // Serial.print(" ");
  //  Serial.print("error pitch: ");
   // Serial.println(nPTgyroX);
   // Serial.print("\t");
   // Serial.print("error yaw: ");
   // Serial.println(gyroZ);
   // Serial.println("===============================");
  //#endif

  //GitLab №1
 /*  Serial.print(f4);
   Serial.print(" ");
   Serial.print(f5);
   Serial.print(" ");
   Serial.print(f6);
   Serial.print(" ");
   Serial.print(f1);
   Serial.print(" ");
   Serial.print(f2);
   Serial.print(" ");
   Serial.print(f3);
   Serial.print(" ");
   Serial.print(nPTgyroX);
   Serial.print(" ");
   Serial.print(nPTgyroY);
   Serial.print(" ");
   Serial.print(nPTgyroZ);
   Serial.print(" ");
   Serial.print(nPTaccelX);
   Serial.print(" ");
   Serial.print(nPTaccelY);
   Serial.print(" ");
   Serial.print(nPTaccelZ);
   Serial.print(" ");
   Serial.print(deg_roll); 
   Serial.print(" ");
   Serial.print(yaw);
   Serial.print(" ");
   Serial.print(TrueYaw);
   Serial.print(" ");
   Serial.print(PercentPowerLF);
   Serial.print(" ");
   Serial.print(PercentPowerRF);
   Serial.print(" ");
   Serial.print(PercentPowerLB);
   Serial.print(" ");
   Serial.print(PercentPowerRB); */

   
} 