#include <Arduino.h>
#include "Fly_sky_iBus.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ESP32CAN.h>

#define LED1 12
#define LED2 14

#define MIN_POWER 3276
#define MAX_POWER 5600

#define POWER_INC 3

#define INTEGRAL_COEFFICIENT 0.6

#define PWM_MOTOR_1 25  // D6 X14 LB
#define PWM_MOTOR_2 32  // D4 X12 RF
#define PWM_MOTOR_3 33  // D5 X13 RB
#define PWM_MOTOR_4 26  // D3 X8  LF

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
const int resolution = 16;

float yaw, pitch, roll;
float deg_yaw, deg_pitch, deg_roll;
float targetRoll, targetPitch, targetYaw;
float errorPitch, errorRoll, errorYaw;
float additionalPowerRF, additionalPowerRB, additionalPowerLF, additionalPowerLB;
float powerRF = 3276.0;
float powerRB = 3276.0;
float powerLF = 3276.0;
float powerLB = 3276.0;
float addPower;
float targetPowerRF, targetPowerRB, targetPowerLF, targetPowerLB;

void timer_interrupt() {
    
}

// --- CAN шина ---
CAN_device_t CAN_cfg;             // Конфигурация CAN
unsigned long previousMillis = 0; // Время последней отправки CAN сообщения
const int interval = 500;         // Интервал отправки CAN сообщений (мс)
const int rx_queue_size = 1;     // Размер буфера приёма
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_23;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_19;

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;
TaskHandle_t Task5;

SemaphoreHandle_t serial_mutex;
SemaphoreHandle_t param_mutex;

void iBusReadTask(void* pvParameters){

  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 10 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    //xSemaphoreTake(serial_mutex, portMAX_DELAY);
    static uint16_t chanel[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 6; ++i)
    {
      chanel_read[i] = iBus.readChannel(i);
      targetRoll = map(chanel_read[0], 1000, 2000, -10, 10);
      targetPitch = map(chanel_read[1], 1000, 2000, -10, 10);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    //xSemaphoreGive(serial_mutex);
  } 
}

void pidRegulatorTask(void* pvParameters){

  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 10 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    //xSemaphoreTake(param_mutex, portMAX_DELAY);
    errorRoll = targetRoll - (int16_t)deg_roll;
    errorPitch = targetPitch - (int16_t)deg_pitch;
    errorYaw = targetYaw - (int16_t)deg_yaw;

    // additionalPowerLB += (errorPitch* 1) + (errorRoll * 1);
    // additionalPowerRB += (errorPitch * 1 - errorRoll * 1);
    // additionalPowerLF += (-(errorPitch * 1) + errorRoll * 1);
    // additionalPowerRF += (-(errorPitch * 1) - errorRoll * 1);

    // additionalPowerLB += (errorPitch* 0.1) + (errorRoll * 0.1);
    // additionalPowerRB += (errorPitch * 0.1 - errorRoll * 0.1);
    // additionalPowerLF += (-(errorPitch * 0.1) + errorRoll * 0.1);
    // additionalPowerRF += (-(errorPitch * 0.1) - errorRoll * 0.1);

    

    if (additionalPowerLB > (powerLB - MIN_POWER) * INTEGRAL_COEFFICIENT) {
      additionalPowerLB = (powerLB - MIN_POWER) * INTEGRAL_COEFFICIENT;
    }
    if (additionalPowerLB < -(powerLB - MIN_POWER) * INTEGRAL_COEFFICIENT) {
      additionalPowerLB = -(powerLB - MIN_POWER) * INTEGRAL_COEFFICIENT;
    }

    if (additionalPowerRB > (powerRB - MIN_POWER) * INTEGRAL_COEFFICIENT) {
      additionalPowerRB = (powerRB - MIN_POWER) * INTEGRAL_COEFFICIENT;
    }
    if (additionalPowerRB < -(powerRB - MIN_POWER) * INTEGRAL_COEFFICIENT) {
      additionalPowerRB = -(powerRB - MIN_POWER) * INTEGRAL_COEFFICIENT;
    }

    if (additionalPowerLF > (powerLF - MIN_POWER) * INTEGRAL_COEFFICIENT) {
      additionalPowerLF = (powerLF - MIN_POWER) * INTEGRAL_COEFFICIENT;
    }
    if (additionalPowerLF < -(powerLF - MIN_POWER) * INTEGRAL_COEFFICIENT) {
      additionalPowerLF = -(powerLF - MIN_POWER) * INTEGRAL_COEFFICIENT;
    }

    if (additionalPowerRF > (powerRF - MIN_POWER) * INTEGRAL_COEFFICIENT) {
      additionalPowerRF = (powerRF - MIN_POWER) * INTEGRAL_COEFFICIENT;
    }
    if (additionalPowerRF < -(powerRF - MIN_POWER) * INTEGRAL_COEFFICIENT) {
      additionalPowerRF = -(powerRF - MIN_POWER) * INTEGRAL_COEFFICIENT;
    }

    targetPowerLB = powerLB + additionalPowerLB;
    targetPowerRF = powerRF + additionalPowerRF;
    targetPowerRB = powerRB + additionalPowerRB;
    targetPowerLF = powerLF + additionalPowerLF;

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}
 

void iBusLoopTask(void* pvParameters){

  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 10 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    iBus.loop();
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void motorsControlTask(void* pvParameters) {
  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 10 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    if ((chanel_read[4] > 1900) && (chanel_read[5] > 1900)) {

      if (chanel_read[2] > 1700) {
        if (powerLB <= (MAX_POWER - POWER_INC)) {
          powerLB += POWER_INC;
        }
        if (powerLF <= MAX_POWER - POWER_INC) {
          powerLF += POWER_INC;
        }
        if (powerRB <= MAX_POWER - POWER_INC) {
          powerRB += (3.0 + 1.5);
        }
        if (powerRF <= MAX_POWER - POWER_INC) {
          powerRF += (3.0 + 1.5);
        }
      }
      if (chanel_read[2] < 1300) {
        if (powerLB >= (MIN_POWER + POWER_INC)) {
          powerLB -= POWER_INC * 2;
        }
        if (powerLF >= MIN_POWER + POWER_INC) {
          powerLF -= POWER_INC * 2;
        }
        if (powerRB >= MIN_POWER + POWER_INC) {
          powerRB -= POWER_INC * 2;
        }
        if (powerRF >= MIN_POWER + POWER_INC) {
          powerRF -= POWER_INC * 2;
      }
    }
      ledcWrite(pwmChannelMotor_1, targetPowerLB); //black usb back   LB  
      ledcWrite(pwmChannelMotor_2, targetPowerRF); //red usb front    RF
      ledcWrite(pwmChannelMotor_3, targetPowerRB); // red usb back    RB
      ledcWrite(pwmChannelMotor_4, targetPowerLF); //black usb front  LF
    }
    else {
      ledcWrite(pwmChannelMotor_1, MIN_POWER);
      ledcWrite(pwmChannelMotor_2, MIN_POWER);
      ledcWrite(pwmChannelMotor_3, MIN_POWER);
      ledcWrite(pwmChannelMotor_4, MIN_POWER);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  } 
}

void canReceiveTask(void* pvParameter) {
    portTickType xLastWakeTime;
    const portTickType xPeriod = ( 5 / portTICK_RATE_MS );
    xLastWakeTime = xTaskGetTickCount();
    BaseType_t xTaskWokenByReceive = pdFALSE;
    for (;;) {
      CAN_frame_t rx_frame;
      if (xQueueReceiveFromISR(CAN_cfg.rx_queue, &rx_frame, &xTaskWokenByReceive) == pdTRUE) {
          if (rx_frame.MsgID == 0x11) {
              memcpy(&roll, &rx_frame.data.u8[0], 4);
              memcpy(&pitch, &rx_frame.data.u8[4], 4);
              deg_roll = roll;
              deg_pitch = pitch;
          }
          else if (rx_frame.MsgID == 0x13) {
              memcpy(&yaw, &rx_frame.data.u8[0], 4);
          }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
      }
      if( xTaskWokenByReceive != pdFALSE ) {
          /* We should switch context so the ISR returns to a different task.
          NOTE:  How this is done depends on the port you are using.  Check
          the documentation and examples for your port. */
          taskYIELD ();
      }
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

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
  ledcSetup(pwmChannelMotor_1, frequency, resolution);
  ledcSetup(pwmChannelMotor_2, frequency, resolution);
  ledcSetup(pwmChannelMotor_3, frequency, resolution);
  ledcSetup(pwmChannelMotor_4, frequency, resolution);
  ledcAttachPin(PWM_MOTOR_1, pwmChannelMotor_1);
  ledcAttachPin(PWM_MOTOR_2, pwmChannelMotor_2);
  ledcAttachPin(PWM_MOTOR_3, pwmChannelMotor_3);
  ledcAttachPin(PWM_MOTOR_4, pwmChannelMotor_4);
  ledcWrite(pwmChannelMotor_1, MIN_POWER);
  ledcWrite(pwmChannelMotor_2, MIN_POWER);
  ledcWrite(pwmChannelMotor_3, MIN_POWER);
  ledcWrite(pwmChannelMotor_4, MIN_POWER);
  // Инициализация двигателей

  delay(2500);

  // Создание мьютексов
  serial_mutex = xSemaphoreCreateMutex();
  param_mutex = xSemaphoreCreateMutex();
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

void loop() {
    Serial.print("roll: ");
    Serial.print(errorRoll);
    Serial.print("\t\t");
    Serial.print("pitch: ");
    Serial.print(errorPitch);
    Serial.print("\t\t");
    Serial.print("yaw: ");
    Serial.println(errorRoll);
    Serial.println("===============================");
    delay(100);
    // Serial.print("powerLB: ");
    // Serial.print(powerLB);
    // Serial.print("\t\t");
    // Serial.print("add power LB: ");
    // Serial.print(additionalPowerLB);
    // Serial.print("\t\t");
    // Serial.print("target power: ");
    // Serial.println(targetPowerLB);
    // Serial.println("===============================");
    // delay(1000);
}

//   //xSemaphoreTake(serial_mutex, portMAX_DELAY);

//   //xSemaphoreGive(serial_mutex);
// }