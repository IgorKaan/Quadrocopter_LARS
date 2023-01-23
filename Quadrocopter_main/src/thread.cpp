#include <Arduino.h>
#include <cmath>
#include "PID.hpp"
#include "TFMPlus.h"
#include "thread.hpp"
#include "config.hpp"
#include "ESP32CAN.h"
#include "Fly_sky_iBus.h"
#include "Modes.hpp"

uint16_t channel_read[6];

Arm_mode arm_m;
// TODO: arm_m.set_range(uint16_t ch_val, uint16_t throttle_val);

float altitude;
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

PIDImpl pidRoll(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 4.5, 2.0, 1.0);
PIDImpl pidPitch(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 4.5, 2.0, 1.0);
PIDImpl pidYaw(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 2.0, 1.0, 0.5);

// PIDImpl pidRoll(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 4.5, 2.0, 0);
// PIDImpl pidPitch(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 4.5, 2.0, 0);
// PIDImpl pidYaw(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 4.0, 2.0, 0);

// PIDImpl pidRoll(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 3.5, 2.0, 0);
// PIDImpl pidPitch(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 3.5, 2.0, 0);
// PIDImpl pidYaw(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 3.5, 2.0, 0);

extern TFMPlus tfmP;


void timer_interrupt() {
    
}

void iBusReadTask(void* pvParameters) {

  portTickType xLastWakeTime;
	const portTickType xPeriod = (IBUS_READ_TASK_HZ / portTICK_RATE_MS);
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    //xSemaphoreTake(serial_mutex, portMAX_DELAY);
    static uint16_t channel[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 6; ++i)
    {
      channel_read[i] = iBus.readChannel(i);
    }
    arm_m.set_value(channel_read[4], channel_read[2], roll, pitch);
    
    targetRoll = map(channel_read[0], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, -TARGET_ANGLE, TARGET_ANGLE);
    targetPitch = map(channel_read[1], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, -TARGET_ANGLE, TARGET_ANGLE);
    targetYaw = map(channel_read[3], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, -TARGET_ANGLE, TARGET_ANGLE);
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    //xSemaphoreGive(serial_mutex);
  } 
}

void pidRegulatorTask(void* pvParameters) {

  portTickType xLastWakeTime;
	const portTickType xPeriod = (PID_REGULATOR_TASK_HZ / portTICK_RATE_MS);
	xLastWakeTime = xTaskGetTickCount();

  // PIDImpl pidRoll(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 4, 1.5, 0.1);
  // PIDImpl pidPitch(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 4, 1.5, 0.1);
  // PIDImpl pidYaw(1, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 3, 2, 0);

  for(;;) {
    //xSemaphoreTake(param_mutex, portMAX_DELAY);
    // errorRoll = targetRoll - deg_roll;
    // errorPitch = targetPitch - deg_pitch;
    // errorYaw = targetYaw - deg_yaw;
    errorRoll = pidRoll.calculate(targetRoll, deg_roll);
    errorPitch = pidPitch.calculate(targetPitch, deg_pitch);
    errorYaw = pidYaw.calculate(targetYaw, deg_yaw);

    // additionalPowerLB += (pidPitch.calculate(targetPitch, pitch) + pidRoll.calculate(targetRoll, roll));
    // additionalPowerRB += (pidPitch.calculate(targetPitch, pitch) - pidRoll.calculate(targetRoll, roll));
    // additionalPowerLF += (-pidPitch.calculate(targetPitch, pitch) + pidRoll.calculate(targetRoll, roll));
    // additionalPowerRF += (-pidPitch.calculate(targetPitch, pitch) - pidRoll.calculate(targetRoll, roll));

    // additionalPowerLB = errorPitch + errorRoll - errorYaw;
    // additionalPowerRB = errorPitch - errorRoll + errorYaw;
    // additionalPowerLF = -errorPitch + errorRoll + errorYaw;
    // additionalPowerRF = -errorPitch - errorRoll - errorYaw;

    additionalPowerLB = errorPitch + errorRoll;
    additionalPowerRB = errorPitch - errorRoll;
    additionalPowerLF = -errorPitch + errorRoll;
    additionalPowerRF = -errorPitch - errorRoll;

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
	const portTickType xPeriod = (IBUS_LOOP_TASK_HZ / portTICK_RATE_MS);
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    iBus.loop();
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void motorsControlTask(void* pvParameters) {
  portTickType xLastWakeTime;
	const portTickType xPeriod = (MOTOR_CONTROL_TASK_HZ / portTICK_RATE_MS);
	xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    if (arm_m.get_value()) {
      //pidRoll.setPcoefficient(map(channel_read[4], 1000, 2000, 1.0, 7.0));
      //pidRoll.setDcoefficient(map(channel_read[5], 1000, 2000, 1, 4));
      // pidPitch.setPcoefficient(6);
      // pidPitch.setDcoefficient(6);

      #ifdef STABILIZE_MODE 
      
      powerLB = map(channel_read[2], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, MIN_POWER, MAX_POWER);
      powerLF = map(channel_read[2], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, MIN_POWER, MAX_POWER);
      powerRB = map(channel_read[2], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, MIN_POWER, MAX_POWER);
      powerRF = map(channel_read[2], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, MIN_POWER, MAX_POWER);

      ledcWrite(PWM_CHANNEL_MOTOR_1, targetPowerLB); //black usb back   LB  
      ledcWrite(PWM_CHANNEL_MOTOR_2, targetPowerRF); //red usb front    RF
      ledcWrite(PWM_CHANNEL_MOTOR_3, targetPowerRB); // red usb back    RB
      ledcWrite(PWM_CHANNEL_MOTOR_4, targetPowerLF); //black usb front  LF
      
      #endif
    }
    else {
      ledcWrite(PWM_CHANNEL_MOTOR_1, MIN_POWER);
      ledcWrite(PWM_CHANNEL_MOTOR_2, MIN_POWER);
      ledcWrite(PWM_CHANNEL_MOTOR_3, MIN_POWER);
      ledcWrite(PWM_CHANNEL_MOTOR_4, MIN_POWER);
      targetPowerLB = MIN_POWER;
      targetPowerRF = MIN_POWER;
      targetPowerRB = MIN_POWER;
      targetPowerLF = MIN_POWER;
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  } 
}

void canReceiveTask(void* pvParameter) {
  portTickType xLastWakeTime;
  const portTickType xPeriod = (CAN_RECEIVE_TASK_HZ / portTICK_RATE_MS);
  xLastWakeTime = xTaskGetTickCount();
  BaseType_t xTaskWokenByReceive = pdFALSE;
  for (;;) {
    CAN_frame_t rx_frame;
    if (xQueueReceiveFromISR(CAN_cfg.rx_queue, &rx_frame, &xTaskWokenByReceive) == pdTRUE) {
        if (rx_frame.MsgID == 0x11) {
            memcpy(&roll, &rx_frame.data.u8[0], sizeof(float));
            memcpy(&pitch, &rx_frame.data.u8[4], sizeof(float));
            deg_roll = roll;
            deg_pitch = pitch;
        }
        else if (rx_frame.MsgID == 0x13) {
            memcpy(&yaw, &rx_frame.data.u8[0], sizeof(float));
            deg_yaw = yaw;
        }       
        else if (rx_frame.MsgID == 0x16) {
            memcpy(&altitude, &rx_frame.data.u8[0], sizeof(float));
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

void TFMiniReadTask(void* pvParameters) {
  int16_t tfDist = 0;    // Distance to object in centimeters
  int16_t tfFlux = 0;    // Strength or quality of return signal
  int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
  portTickType xLastWakeTime;
  const portTickType xPeriod = (TFMINI_RECEIVE_TASK_HZ / portTICK_RATE_MS);
  xLastWakeTime = xTaskGetTickCount();
  BaseType_t xTaskWokenByReceive = pdFALSE;
  for (;;) {
    // tfmP.getData(tfDist, tfFlux, tfTemp);
    // Serial.print("Distance: ");
    // Serial.print(tfDist);
    // Serial.print("\t");
    // Serial.print("Flux: ");
    // Serial.print(tfFlux);
    // Serial.print("\t");
    // Serial.print("Temp: ");
    // Serial.print(tfTemp);
    // Serial.print("\t");
    // Serial.println();
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
 
}