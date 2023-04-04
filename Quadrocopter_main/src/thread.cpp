#include <Arduino.h>
#include "PID.hpp"
#include "Biquad.h"
#include "TFMPlus.h"
#include "thread.hpp"
#include "config.hpp"
#include "ESP32CAN.h"
#include "Fly_sky_iBus.h"
extern "C" {
  #include "madgwickFilter.h"
}

extern SemaphoreHandle_t serial_mutex;
extern SemaphoreHandle_t param_mutex;

uint16_t channel_read[6];

Biquad *lpFilterGyroX = new Biquad();
Biquad *lpFilterGyroY = new Biquad();
Biquad *lpFilterGyroZ = new Biquad();
Biquad *lpFilterAccelX = new Biquad();
Biquad *lpFilterAccelY = new Biquad();
Biquad *lpFilterAccelZ = new Biquad();

Biquad *nFilterGyroX = new Biquad();
Biquad *nFilterGyroY = new Biquad();
Biquad *nFilterGyroZ = new Biquad();
Biquad *nFilterAccelX = new Biquad();
Biquad *nFilterAccelY = new Biquad();
Biquad *nFilterAccelZ = new Biquad();

float gxPT1, gyPT1, gzPT1, axPT1, ayPT1, azPT1;

float accelXRaw;

float K = 0.9;

float pt1FilterApplyGX(float input)
{
  gxPT1 = gxPT1 + K * (input - gxPT1);
  return gxPT1;
}

float pt1FilterApplyGY(float input)
{
  gyPT1 = gyPT1 + K * (input - gyPT1);
  return gyPT1;
}

float pt1FilterApplyGZ(float input)
{
  gzPT1 = gzPT1 + K * (input - gzPT1);
  return gzPT1;
}

float pt1FilterApplyAX(float input)
{
  axPT1 = axPT1 + K * (input - axPT1);
  return axPT1;
}

float pt1FilterApplyAY(float input)
{
  ayPT1 = ayPT1 + K * (input - ayPT1);
  return ayPT1;
}

float pt1FilterApplyAZ(float input)
{
  azPT1 = azPT1 + K * (input - azPT1);
  return azPT1;
}

float altitude;
float yaw, pitch, roll;
float deg_yaw, deg_pitch, deg_roll;
float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;

float ngyroX, ngyroY, ngyroZ, naccelX, naccelY, naccelZ;

float nPTgyroX, nPTgyroY, nPTgyroZ, nPTaccelX, nPTaccelY, nPTaccelZ, lPTgyroX, lPTgyroY, lPTgyroZ, lPTaccelX, lPTaccelY, lPTaccelZ;

float targetRoll, targetPitch, targetYaw;
float errorRoll, errorPitch, errorYaw;
float additionalPowerRF, additionalPowerRB, additionalPowerLF, additionalPowerLB;
float powerRF = MIN_POWER;
float powerRB = MIN_POWER;
float powerLF = MIN_POWER;
float powerLB = MIN_POWER;
float targetPowerRF, targetPowerRB, targetPowerLF, targetPowerLB;

PIDImpl pidRoll(0.01, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 1.5, 1.06, 1.5);        //1.5, 0.7, 1.5
PIDImpl pidPitch(0.01, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 1.5, 1.06, 1.5);       //1.5, 0.7, 1.5
PIDImpl pidYaw(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 5, 0.5, 5);        //4.5
// PIDImpl pidRoll(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 4.5, 2.0, 0);
// PIDImpl pidPitch(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 4.5, 2.0, 0);
// PIDImpl pidYaw(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 4.0, 2.0, 0);

// PIDImpl pidRoll(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 3.5, 2.0, 0);
// PIDImpl pidPitch(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 3.5, 2.0, 0);
// PIDImpl pidYaw(0.001, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 3.5, 2.0, 0);

extern TFMPlus tfmP; 


void timer_interrupt() {
    
}

uint8_t buffer[27] {0,};
float f1, f2, f3, f4, f5, f6;

void filterTask(void* pvParameters) { 
  portTickType xLastWakeTime;
	const portTickType xPeriod = (FILTER_TASK_HZ / portTICK_RATE_MS);
	xLastWakeTime = xTaskGetTickCount();
  for(;;) {

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    //xSemaphoreGive(serial_mutex);
  } 
}


void UARTReadTask(void* pvParameters) {
  portTickType xLastWakeTime;
  const portTickType xPeriod = (UART_RECEIVE_TASK_HZ / portTICK_RATE_MS);
  xLastWakeTime = xTaskGetTickCount();
  BaseType_t xTaskWokenByReceive = pdFALSE;
  lpFilterGyroX->setBiquad(bq_type_lowpass, 0.075, 0.707, 0);
  lpFilterGyroY->setBiquad(bq_type_lowpass, 0.075, 0.707, 0);
  lpFilterGyroZ->setBiquad(bq_type_lowpass, 0.075, 0.707, 0);
  lpFilterAccelX->setBiquad(bq_type_lowpass, 0.075, 0.707, 0);
  lpFilterAccelY->setBiquad(bq_type_lowpass, 0.075, 0.707, 0);
  lpFilterAccelZ->setBiquad(bq_type_lowpass, 0.075, 0.707, 0);

  nFilterGyroX -> setBiquad(bq_type_notch, 0.5, 2, 0);  // 0.5, 2, 0
  nFilterGyroY -> setBiquad(bq_type_notch, 0.5, 2, 0);
  nFilterGyroZ -> setBiquad(bq_type_notch, 0.5, 2, 0);
  nFilterAccelX -> setBiquad(bq_type_notch, 0.5, 2, 0);
  nFilterAccelY -> setBiquad(bq_type_notch, 0.5, 2, 0);
  nFilterAccelZ -> setBiquad(bq_type_notch, 0.5, 2, 0);

  for (;;) {

    if (Serial1.available()) {
      //Serial.println("receive");
      uint8_t sByte = Serial1.read();
      //Serial.println(sByte);
      if (sByte == 83) {
        //Serial.println("receive");
        //Serial.println(millis());
        Serial1.readBytes(buffer, 27);
        if ((buffer[0] == 83) && (buffer[25] == 69) && (buffer[26] == 69)) {
          memcpy(&f1, &buffer[1], sizeof(float));
          memcpy(&f2, &buffer[5], sizeof(float));
          memcpy(&f3, &buffer[9], sizeof(float));
          memcpy(&f4, &buffer[13], sizeof(float));
          memcpy(&f5, &buffer[17], sizeof(float));
          memcpy(&f6, &buffer[21], sizeof(float));
        }
        // memcpy(&f1, &buffer[0], sizeof(float));
        // memcpy(&f2, &buffer[4], sizeof(float));
        // memcpy(&f3, &buffer[8], sizeof(float));
        // memcpy(&f4, &buffer[12], sizeof(float));
        // memcpy(&f5, &buffer[16], sizeof(float));
        // memcpy(&f6, &buffer[20], sizeof(float));
       // accelXRaw = f1;+

       //gyroX = lpFilterGyroX -> process(f4);
       // gyroY = lpFilterGyroY -> process(f5);
        // gyroZ = lpFilterGyroZ -> process(f6);
       // accelX = lpFilterAccelX -> process(f1);
       // accelY = lpFilterAccelY -> process(f2);
       //accelZ = lpFilterAccelZ -> process(f3);
        // nPTgyroX = pt1FilterApplyGX(gyroX);
        // nPTgyroY = pt1FilterApplyGY(gyroY);
        // nPTgyroZ = pt1FilterApplyGZ(gyroZ);
        // nPTaccelX = pt1FilterApplyAX(accelX);
        // nPTaccelY = pt1FilterApplyAY(accelY);
        // nPTaccelZ = pt1FilterApplyAZ(accelZ);

      lPTgyroX = lpFilterGyroX ->process(f4);
      lPTgyroY = lpFilterGyroY ->process(f5);
      lPTgyroZ = lpFilterGyroZ ->process(f6);
      lPTaccelX = lpFilterAccelX ->process(f1);
      lPTaccelY = lpFilterAccelY ->process(f2);
      lPTaccelZ = lpFilterAccelZ ->process(f3);
      nPTgyroX = nFilterGyroX ->process(lPTgyroX);
      nPTgyroY = nFilterGyroY ->process(lPTgyroY);
      nPTgyroZ = nFilterGyroZ ->process(lPTgyroZ);
      nPTaccelX = nFilterAccelX ->process(lPTaccelX);
      nPTaccelY = nFilterAccelY ->process(lPTaccelY);
      nPTaccelZ = nFilterAccelZ ->process(lPTaccelZ);


      // nPTgyroX = lpFilterGyroX -> process(f4);
      //  nPTgyroY = lpFilterGyroY -> process(f5);
      // nPTgyroZ = lpFilterGyroZ -> process(f6);
      //nPTaccelX = lpFilterAccelX -> process(f1);
       //nPTaccelY = lpFilterAccelY -> process(f2);
       // nPTaccelZ = lpFilterAccelZ -> process(f3);

        imu_filter_rp(-nPTaccelX, -nPTaccelY, -nPTaccelZ, nPTgyroX, nPTgyroY, 0); //Данил. Внедрил nPTgyroZ

        //imu_filter_y(accelX_average, accelY_average, accelZ_average, gyroX_average, gyroY_average, gyroZ_average);
        yaw = 0;
        q_est_rp.q4 = 0;
        eulerAngles(q_est_rp, &roll, &pitch, &yaw);
        deg_roll = roll;
        deg_pitch = pitch;
        deg_yaw = yaw;
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
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
    
    targetPitch = map(channel_read[0], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, -TARGET_ANGLE, TARGET_ANGLE);
    targetRoll= map(channel_read[1], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, -TARGET_ANGLE, TARGET_ANGLE);
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
    // xSemaphoreTake(param_mutex, portMAX_DELAY);
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

    // additionalPowerLB = errorPitch + errorRoll;
    // additionalPowerRB = errorPitch - errorRoll;
    // additionalPowerLF = -errorPitch + errorRoll;
    // additionalPowerRF = -errorPitch - errorRoll;

    additionalPowerLB = errorPitch + errorRoll + errorYaw;
    additionalPowerRB = -errorPitch + errorRoll - errorYaw;
    additionalPowerLF = errorPitch - errorRoll - errorYaw;
    additionalPowerRF = -errorPitch - errorRoll + errorYaw;

    
    // additionalPowerLB = errorPitch + errorRoll;
    // additionalPowerRB = -errorPitch + errorRoll;
    // additionalPowerLF = errorPitch - errorRoll;
    // additionalPowerRF = -errorPitch - errorRoll;

    // additionalPowerLB = errorPitch;
    // additionalPowerRB = -errorPitch;
    // additionalPowerLF = errorPitch;
    // additionalPowerRF = -errorPitch;


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

    if ((channel_read[4] > 1900) && (channel_read[5] > 1900)) {

      #ifdef STABILIZE_MODE 
      
      powerLB = map(channel_read[2], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, MIN_POWER, MAX_POWER);
      powerLF = map(channel_read[2], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, MIN_POWER, MAX_POWER);
      powerRB = map(channel_read[2], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, MIN_POWER, MAX_POWER);
      powerRF = map(channel_read[2], MIN_JOY_OUTPUT, MAX_JOY_OUTPUT, MIN_POWER, MAX_POWER);

      ledcWrite(PWM_CHANNEL_MOTOR_1, targetPowerLB); //black usb back   LB  
      ledcWrite(PWM_CHANNEL_MOTOR_3, targetPowerRF); //red usb front    RF
      ledcWrite(PWM_CHANNEL_MOTOR_4, targetPowerRB); // red usb back    RB
      ledcWrite(PWM_CHANNEL_MOTOR_2, targetPowerLF); //black usb front  LF
      
      #endif
    }
    else {

      ledcWrite(PWM_CHANNEL_MOTOR_1, MIN_POWER);
      ledcWrite(PWM_CHANNEL_MOTOR_2, MIN_POWER);
      ledcWrite(PWM_CHANNEL_MOTOR_3, MIN_POWER);
      ledcWrite(PWM_CHANNEL_MOTOR_4, MIN_POWER);
      // ledcWrite(PWM_CHANNEL_MOTOR_1, 3400); // RF
      // ledcWrite(PWM_CHANNEL_MOTOR_2, 3400); // LF
      // ledcWrite(PWM_CHANNEL_MOTOR_3, 3400); // RB
      // ledcWrite(PWM_CHANNEL_MOTOR_4, 3400); // LB
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
    // if (xQueueReceiveFromISR(CAN_cfg.rx_queue, &rx_frame, &xTaskWokenByReceive) == pdTRUE) {
    //     if (rx_frame.MsgID == 0x11) {
    //         memcpy(&roll, &rx_frame.data.u8[0], sizeof(float));
    //         memcpy(&pitch, &rx_frame.data.u8[4], sizeof(float));
    //         deg_roll = roll;
    //         deg_pitch = pitch;
    //     }
    //     else if (rx_frame.MsgID == 0x13) {
    //         memcpy(&yaw, &rx_frame.data.u8[0], sizeof(float));
    //         deg_yaw = yaw;
    //     }       
    //     else if (rx_frame.MsgID == 0x16) {
    //         memcpy(&altitude, &rx_frame.data.u8[0], sizeof(float));
    //     }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    // }
    // if( xTaskWokenByReceive != pdFALSE ) {
    //     /* We should switch context so the ISR returns to a different task.
    //     NOTE:  How this is done depends on the port you are using.  Check
    //     the documentation and examples for your port. */
    //     taskYIELD ();
    // }
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