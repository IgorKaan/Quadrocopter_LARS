#include <Arduino.h>
#include "Fly_sky_iBus.h"

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

long impulse_time[4];

void iBus_read_fun();

void timer_interrupt() {
    
}

void setup()
{
  Serial.begin(115200);

  iBus.begin(Serial2);

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
  /// --- - ---
}

void loop()
{
  static ulong time = millis();
  if (millis() - time > 200U)
  {
  }
  iBus_read_fun();
}

const uint16_t max_speed = 350;
const uint16_t min_speed = 50;
const uint16_t max_revers_speed = 135;
const uint16_t min_revers_speed = 75;

bool forward = false;
bool backward = false;
bool stop_US = false;

int32_t left_speed = 0;
int32_t right_speed = 0;
bool left_revers = false;
bool right_revers = false;

void iBus_read_fun()
{
  static uint16_t chanel[6] = {0, 0, 0, 0, 0, 0};
  uint16_t chanel_read[6];
  iBus.loop();

  bool signal = false;
  for (int i = 0; i < 6; ++i)
  {
    chanel_read[i] = iBus.readChannel(i);

    if ((chanel[i] <= (chanel_read[i] - 5)) || ((chanel[i] >= (chanel_read[i] + 5))))
      signal = true;
  }
  Serial.print("Channel 1 = ");
  Serial.println(chanel_read[0], DEC);
  Serial.print("Channel 2 = ");
  Serial.println(chanel_read[1], DEC);
  Serial.print("Channel 3 = ");
  Serial.println(chanel_read[2], DEC);
  Serial.print("Channel 4 = ");
  Serial.println(chanel_read[3], DEC);
  Serial.print("Channel 5 = ");
  Serial.println(chanel_read[4], DEC);
  Serial.print("Channel 6 = ");
  Serial.println(chanel_read[5], DEC);
  Serial.println("==================================================");
  delay(500);
}
