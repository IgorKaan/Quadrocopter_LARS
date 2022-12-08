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
      targetRoll = map(chanel_read[0], 1000, 2000, -10, 10);
      targetPitch = map(chanel_read[1], 1000, 2000, -10, 10);
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    //xSemaphoreGive(serial_mutex);
  } 
}

void pidRegulatorTask(void* pvParameters) {

  portTickType xLastWakeTime;
	const portTickType xPeriod = ( 5 / portTICK_RATE_MS );
	xLastWakeTime = xTaskGetTickCount();

  PIDImpl pidRoll(0.01, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 0.3, 0.1, 0.0001);
  PIDImpl pidPitch(0.01, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 0.3, 0.1, 0.0001);
  PIDImpl pidYaw(0.01, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 0.3, 0.1, 0.0001);

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

    additionalPowerLB += (errorPitch * 1) + (errorRoll * 1);
    additionalPowerRB += (errorPitch * 1 - errorRoll * 1);
    additionalPowerLF += (-(errorPitch * 1) + errorRoll * 1);
    additionalPowerRF += (-(errorPitch * 1) - errorRoll * 1);

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
          powerRB += (POWER_INC);
        }
        if (powerRF <= MAX_POWER - POWER_INC) {
          powerRF += (POWER_INC);
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