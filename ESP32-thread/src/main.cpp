#include <Arduino.h>

#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void Task1code(void* pvParameters);
void Task2code(void* pvParameters);

volatile int count = 0;

TaskHandle_t Task1;
TaskHandle_t Task2;

SemaphoreHandle_t serial_mutex;

void Task1code( void * pvParameters ){

  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 1 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  for(;;){
    //xSemaphoreTake(serial_mutex, portMAX_DELAY);
    Serial.print("Task1 running on core ");
    Serial.println(xPortGetCoreID());
    count++;
    Serial.print("Count = ");
    Serial.println(count);
    //xSemaphoreGive(serial_mutex);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  } 
}
 

void Task2code( void * pvParameters ){

  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 1 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  for(;;){
    //xSemaphoreTake(serial_mutex, portMAX_DELAY);
    Serial.print("Task2 running on core ");
    Serial.println(xPortGetCoreID());
    count++;
    Serial.print("Count = ");
    Serial.println(count);
    //xSemaphoreGive(serial_mutex);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void setup() {
  Serial.begin(115200);

  Serial.print("setup() running on core ");

  Serial.println(xPortGetCoreID());

  serial_mutex = xSemaphoreCreateMutex();

  BaseType_t t1 = xTaskCreatePinnedToCore(Task1code, "Task1", 100000, NULL, 1, &Task1, 0);
  BaseType_t t2 = xTaskCreatePinnedToCore(Task2code, "Task2", 100000, NULL, 1, &Task2, 1);
}

void loop() {
  Serial.print("loop() running on core ");
 
  Serial.println(xPortGetCoreID());
  delay(3000);
}