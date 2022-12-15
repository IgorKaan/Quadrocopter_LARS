#include <Arduino.h>

#include "PID.hpp"
#include "thread.hpp"
#include "config.hpp"
#include "ESP32CAN.h"
#include "Fly_sky_iBus.h"

uint16_t chanel_read[6];

float yaw, pitch, roll;
float deg_yaw, deg_pitch, deg_roll;
float targetRoll, targetPitch, targetYaw;
float errorRoll, errorPitch, errorYaw;
float additionalPowerRF, additionalPowerRB, additionalPowerLF, additionalPowerLB;
float powerRF = MIN_POWER;
float powerRB = MIN_POWER;
float powerLF = MIN_POWER;
float powerLB = MIN_POWER;
float targetPowerRF, targetPowerRB, targetPowerLF, targetPowerLB;

void timer_interrupt() {
    
}

void iBusReadTask(void* pvParameters) {

  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 10 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    //xSemaphoreTake(serial_mutex, portMAX_DELAY);
    static uint16_t chanel[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 6; ++i)
    {
      chanel_read[i] = iBus.readChannel(i);
    }
    
    targetRoll = map(chanel_read[0], 1000, 2000, -TARGET_ANGLE, TARGET_ANGLE);
    targetPitch = map(chanel_read[1], 1000, 2000, -TARGET_ANGLE, TARGET_ANGLE);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    //xSemaphoreGive(serial_mutex);
  } 
}

void pidRegulatorTask(void* pvParameters) {

  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 1 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  PIDImpl pidRoll(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 5, 1.5, 0.1);
  PIDImpl pidPitch(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 5, 1.5, 0.1);
  PIDImpl pidYaw(1, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 3, 2, 0);

  for(;;) {
    //xSemaphoreTake(param_mutex, portMAX_DELAY);
    // errorRoll = targetRoll - deg_roll;
    // errorPitch = targetPitch - deg_pitch;
    // errorYaw = targetYaw - deg_yaw;
    errorRoll = pidRoll.calculate(targetRoll, deg_roll);
    errorPitch = pidPitch.calculate(targetPitch, deg_pitch);
    errorYaw = targetYaw - deg_yaw;

    // additionalPowerLB += (pidPitch.calculate(targetPitch, pitch) + pidRoll.calculate(targetRoll, roll));
    // additionalPowerRB += (pidPitch.calculate(targetPitch, pitch) - pidRoll.calculate(targetRoll, roll));
    // additionalPowerLF += (-pidPitch.calculate(targetPitch, pitch) + pidRoll.calculate(targetRoll, roll));
    // additionalPowerRF += (-pidPitch.calculate(targetPitch, pitch) - pidRoll.calculate(targetRoll, roll));

    additionalPowerLB = (errorPitch * 1) + (errorRoll * 1);
    additionalPowerRB = (errorPitch * 1 - errorRoll * 1);
    additionalPowerLF = (-(errorPitch * 1) + errorRoll * 1);
    additionalPowerRF = (-(errorPitch * 1) - errorRoll * 1);

    if ((powerLB + additionalPowerLB) > MAX_POWER) {
      targetPowerLB = MAX_POWER;
    }
    if ((powerLB + additionalPowerLB) < MIN_POWER) {
      targetPowerLB = MIN_POWER;
    }
    targetPowerLB = powerLB + additionalPowerLB;

    if ((powerRF + additionalPowerRF) > MAX_POWER) {
      targetPowerRF = MAX_POWER;
    }
    if ((powerRF + additionalPowerRF) < MIN_POWER) {
      targetPowerRF = MIN_POWER;
    }
    targetPowerRF = powerRF + additionalPowerRF;

    if ((powerRB + additionalPowerRB) > MAX_POWER) {
      targetPowerRB = MAX_POWER;
    }
    if ((powerRB + additionalPowerRB) < MIN_POWER) {
      targetPowerRB = MIN_POWER;
    }
    targetPowerRB = powerRB + additionalPowerRB;

    if ((powerLF + additionalPowerLF) > MAX_POWER) {
      targetPowerLF = MAX_POWER;
    }
    if ((powerLF + additionalPowerLB) < MIN_POWER) {
      targetPowerLF = MIN_POWER;
    }
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
	const portTickType xPeriod = ( 1 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    if ((chanel_read[4] > 1900) && (chanel_read[5] > 1900)) {

      powerLB = map(chanel_read[2], 1100, 1900, MIN_POWER, MAX_POWER);
      powerLF = map(chanel_read[2], 1100, 1900, MIN_POWER, MAX_POWER);
      powerRB = map(chanel_read[2], 1100, 1900, MIN_POWER, MAX_POWER);
      powerRF = map(chanel_read[2], 1100, 1900, MIN_POWER, MAX_POWER);

      ledcWrite(PWM_CHANNEL_MOTOR_1, targetPowerLB); //black usb back   LB  
      ledcWrite(PWM_CHANNEL_MOTOR_2, targetPowerRF); //red usb front    RF
      ledcWrite(PWM_CHANNEL_MOTOR_3, targetPowerRB); // red usb back    RB
      ledcWrite(PWM_CHANNEL_MOTOR_4, targetPowerLF); //black usb front  LF
    }
    else {
      ledcWrite(PWM_CHANNEL_MOTOR_1, MIN_POWER);
      ledcWrite(PWM_CHANNEL_MOTOR_2, MIN_POWER);
      ledcWrite(PWM_CHANNEL_MOTOR_3, MIN_POWER);
      ledcWrite(PWM_CHANNEL_MOTOR_4, MIN_POWER);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  } 
}

void canReceiveTask(void* pvParameter) {
    portTickType xLastWakeTime;
    const portTickType xPeriod = ( 1 / portTICK_RATE_MS );
    xLastWakeTime = xTaskGetTickCount();
    BaseType_t xTaskWokenByReceive = pdFALSE;
    for (;;) {
      CAN_frame_t rx_frame;
      if (xQueueReceiveFromISR(CAN_cfg.rx_queue, &rx_frame, &xTaskWokenByReceive) == pdTRUE) {
          if (rx_frame.MsgID == 0x11) {
              memcpy(&roll, &rx_frame.data.u8[0], 4);
              memcpy(&pitch, &rx_frame.data.u8[4], 4);
              deg_roll = (roll);
              deg_pitch = (pitch);
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