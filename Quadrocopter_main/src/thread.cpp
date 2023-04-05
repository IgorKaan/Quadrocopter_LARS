#include <Arduino.h>
#include "PID.hpp"
#include "Biquad.h"
#include "TFMPlus.h"
#include "thread.hpp"
#include "config.hpp"
#include "ESP32CAN.h"
#include "Fly_sky_iBus.h"
//  #include "Chebishev.h"
#include "Chebishev.h"
#include <LowPassFilter.h>
#include <PT1.h>
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
float altitude;
float yaw, pitch, roll, TrueYaw, ErrorYaw, SumErrorsYaw, SumRiseErrorYaw, RiseErrorYaw, OldSumErrorsYaw;
float deg_yaw, deg_pitch, deg_roll;
float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
extern int c;
float ngyroX, ngyroY, ngyroZ, naccelX, naccelY, naccelZ;
float BiquadNotchAccelX, BiquadNotchAccelY, BiquadNotchAccelZ, BiquadNotchGyroX, BiquadNotchGyroY, BiquadNotchGyroZ,  BiquadLowPassGyroX,  BiquadLowPassGyroY,  BiquadLowPassGyroZ, BiquadLowPassAccelX, BiquadLowPassAccelY, BiquadLowPassAccelZ;
float SumErrorsGyroX, SumErrorsGyroY, SumErrorsGyroZ, ErrorGyroX, ErrorGyroY, ErrorGyroZ;
float SumErrorsAccelX, SumErrorsAccelY, SumErrorsAccelZ, ErrorAccelX, ErrorAccelY, ErrorAccelZ;
float targetRoll, targetPitch, targetYaw;
float errorRoll, errorPitch, errorYaw;
float additionalPowerRF, additionalPowerRB, additionalPowerLF, additionalPowerLB;
float ChebishevGyroX, ChebishevGyroY, ChebishevGyroZ, ChebishevAccelX, ChebishevAccelY, ChebishevAccelZ;
float LowPassGyroX, LowPassGyroY, LowPassGyroZ, LowPassAccelX, LowPassAccelY, LowPassAccelZ;
float PT1GyroX, PT1GyroY, PT1GyroZ, PT1AccelX, PT1AccelY, PT1AccelZ;
float powerRF = MIN_POWER;
float powerRB = MIN_POWER;
float powerLF = MIN_POWER;
float powerLB = MIN_POWER;
float targetPowerRF, targetPowerRB, targetPowerLF, targetPowerLB;
float TrueGyroX, TrueGyroY, TrueGyroZ, TrueAccelX, TrueAccelY, TrueAccelZ;

PIDImpl pidRoll(0.01, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 1.3, 1.3, 2);        //1.5, 0.7, 1.5
PIDImpl pidPitch(0.01, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 1.3, 1.3, 2);       //1.5, 0.7, 1.5
PIDImpl pidYaw(0.01, PID_OUTPUT, -PID_OUTPUT, PID_I_MAX, PID_I_MIN, 8, 0.4, 0);        //4.5

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
  lpFilterGyroX->setBiquad(bq_type_lowpass, 0.07, 0.707, 0);
  lpFilterGyroY->setBiquad(bq_type_lowpass, 0.07, 0.707, 0);
  lpFilterGyroZ->setBiquad(bq_type_lowpass, 0.07, 0.707, 0);
  lpFilterAccelX->setBiquad(bq_type_lowpass, 0.07, 0.707, 0);
  lpFilterAccelY->setBiquad(bq_type_lowpass, 0.07, 0.707, 0);
  lpFilterAccelZ->setBiquad(bq_type_lowpass, 0.07, 0.707, 0);

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
//БИКВАДРАТНЫЙ ФИЛЬТР НИЖНИХ ЧАСТОТ
      BiquadLowPassGyroX = lpFilterGyroX ->process(f4);
      BiquadLowPassGyroY = lpFilterGyroY ->process(f5);
      BiquadLowPassGyroZ = lpFilterGyroZ ->process(f6);
      BiquadLowPassAccelX = lpFilterAccelX ->process(f1);
      BiquadLowPassAccelY = lpFilterAccelY ->process(f2);
      BiquadLowPassAccelZ = lpFilterAccelZ ->process(f3); 
// ФИЛЬТР НИЖНИХ ЧАСТОТ ЧЕБЫШЕВА
/*ChebishevGyroX = ChebishevMathGyroX(f4);    
ChebishevGyroY = ChebishevMathGyroY(f5);
ChebishevGyroZ = ChebishevMathGyroZ(f6);
ChebishevAccelX = ChebishevMathAccelX(f1);
ChebishevAccelY = ChebishevMathAccelY(f2);
ChebishevAccelZ = ChebishevMathAccelZ(f3);*/
// БОМЖ-ФИЛЬТР НИЖНИХ ЧАСТОТ:
/*LowPassGyroX = LowPassMathGX(f4);
LowPassGyroY = LowPassMathGY(f5);
LowPassGyroZ = LowPassMathGZ(f6);
LowPassAccelX = LowPassMathAX(f1);
LowPassAccelY = LowPassMathAY(f2);
LowPassAccelZ = LowPassMathAZ(f3); */
// PT1-ФИЛЬТР НИЖНИХ ЧАСТОТ:
/*PT1GyroX = pt1FilterApplyGX(f4);
PT1GyroY = pt1FilterApplyGY(f5);
PT1GyroZ = pt1FilterApplyGZ(f6);
PT1AccelX = pt1FilterApplyAX(f1);
PT1AccelY = pt1FilterApplyAY(f2);
PT1AccelZ = pt1FilterApplyAZ(f3);*/
// БИКВАДРАТНЫЙ РЕЖЕКТОРНЫЙ ФИЛЬТР ДЛЯ БИКВАДРАТНОГО ФИЛЬТРА НИЖНИХ ЧАСТОТ:
      BiquadNotchGyroX = nFilterGyroX ->process(BiquadLowPassGyroX);
      BiquadNotchGyroY = nFilterGyroY ->process(BiquadLowPassGyroY);
      BiquadNotchGyroZ = nFilterGyroZ ->process(BiquadLowPassGyroZ);
      BiquadNotchAccelX = nFilterAccelX ->process(BiquadLowPassAccelX);
      BiquadNotchAccelY = nFilterAccelY ->process(BiquadLowPassAccelY);
      BiquadNotchAccelZ = nFilterAccelZ ->process(BiquadLowPassAccelZ); 
// БИКВАДРАТНЫЙ РЕЖЕКТОРНЫЙ ФИЛЬТР ДЛЯ ФИЛЬТРА НИЖНИХ ЧАСТОТ ЧЕБЫШЕВА:
     /* BiquadNotchGyroX = nFilterGyroX ->process(ChebishevGyroX);
      BiquadNotchGyroY = nFilterGyroY ->process(ChebishevGyroY);
      BiquadNotchGyroZ = nFilterGyroZ ->process(ChebishevGyroZ);
      BiquadNotchAccelX = nFilterAccelX ->process(ChebishevAccelX);
      BiquadNotchAccelY = nFilterAccelY ->process(ChebishevAccelY);
      BiquadNotchAccelZ = nFilterAccelZ ->process(ChebishevAccelZ); */
// БИКВАДРАТНЫЙ РЕЖЕКТОРНЫЙ ФИЛЬТР ДЛЯ ОБЫЧНОГО ФИЛЬТРА НИЖНИХ ЧАСТОТ:
     /* BiquadNotchGyroX = nFilterGyroX ->process(LowPassGyroX);
      BiquadNotchGyroY = nFilterGyroY ->process(LowPassGyroY);
      BiquadNotchGyroZ = nFilterGyroZ ->process(LowPassGyroZ);
      BiquadNotchAccelX = nFilterAccelX ->process(LowPassAccelX);
      BiquadNotchAccelY = nFilterAccelY ->process(LowPassAccelY);
      BiquadNotchAccelZ = nFilterAccelZ ->process(LowPassAccelZ); */
// БИКВАДРАТНЫЙ РЕЖЕКТОРНЫЙ ФИЛЬТР ДЛЯ PT-1 ФИЛЬТРА НИЖНИХ ЧАСТОТ:
     /* BiquadNotchGyroX = nFilterGyroX ->process(PT1GyroX);
      BiquadNotchGyroY = nFilterGyroY ->process(PT1GyroY);
      BiquadNotchGyroZ = nFilterGyroZ ->process(PT1GyroZ);
      BiquadNotchAccelX = nFilterAccelX ->process(PT1AccelX);
      BiquadNotchAccelY = nFilterAccelY ->process(PT1AccelY);
      BiquadNotchAccelZ = nFilterAccelZ ->process(PT1AccelZ); */
// КОРРЕКТИРОВКА ДАННЫХ С УЧЕТОМ КАЛИБРОВКИ:
      TrueGyroX = BiquadNotchGyroX - ErrorGyroX;
      TrueGyroY = BiquadNotchGyroY - ErrorGyroY;
      TrueGyroZ = BiquadNotchGyroZ - ErrorGyroZ;
      TrueAccelX = BiquadNotchAccelX - ErrorAccelX; 
      TrueAccelY = BiquadNotchAccelY - ErrorAccelY;
      TrueAccelZ = BiquadNotchAccelZ - ErrorAccelZ; 

        imu_filter_rp(-TrueAccelX, -TrueAccelY, -TrueAccelZ, TrueGyroX, TrueGyroY, TrueGyroZ);
        //TrueYaw = yaw - ErrorYaw;
        eulerAngles(q_est_rp, &roll, &pitch, &yaw);
        deg_roll = roll;
        deg_pitch = pitch;
       // TrueYaw = yaw - ErrorYaw;
        deg_yaw = yaw;
        //if(c == 1000)
       // {
       // delay(5);
      //  ErrorYaw += RiseErrorYaw;
      //  }
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

    // additionalPowerLB = errorPitch + errorRoll;
    // additionalPowerRB = errorPitch - errorRoll;
    // additionalPowerLF = -errorPitch + errorRoll;
    // additionalPowerRF = -errorPitch - errorRoll;

    additionalPowerLB = errorPitch + errorRoll - errorYaw;
    additionalPowerRB = -errorPitch + errorRoll + errorYaw;
    additionalPowerLF = errorPitch - errorRoll  + errorYaw;
    additionalPowerRF = -errorPitch - errorRoll - errorYaw;

    
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