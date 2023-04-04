/*#include <Arduino.h>
#include <GoodMorningCopter.h>
extern float f3, f4, f5, f6, accelZ;
float Time, OldTime;
void WakeUp();
void EndCalibration();
void Flag();
uint8_t flag = 1;
void WakeUp()
{
 ledcSetup(4, 500, 8); //256
 ledcAttachPin(27, 4);
 ledcWrite(4, 200);
 delay(1000);
 ledcWrite(4, 0);
}
void ReadyToCalibration()
{
    Serial.println(f3);
    ledcSetup(4, 1000, 8); //256
    ledcAttachPin(27, 4);
    OldTime = millis();
    Serial.println("Bad");
    Serial.println(abs(f3));
    while((abs(f3) > 9) && (abs(f3) < 11)) // (-1 < f4 < 1)&&(-1 < f5 <  1)&&(-1 < f6 < 1))
    {
      Serial.println("0");
        Time = millis() - OldTime;
        if (abs(f4) >= 0)
        {
          Serial.println("1");
        if (abs(f4) <= 0.07)
        {
          Serial.println("2");
        if (abs(f5) >= 0)
        {
          Serial.println("3");
        if (abs(f5) <= 0.07)
        {
          Serial.println("4");
        if (abs(f6) >= 0)
        {
          Serial.println("5");
        if (abs(f6) <= 0.07)
        {
          Serial.println("6");
        if (Time >= 3000)
        {
          Serial.println(f3);
        Serial.println("Hi");
        ledcWrite(4, 200);
        delay(1000);
        ledcWrite(4, 0);
        Time = 27000000;
        Flag();
        //elay(20000);
        break;
        }
        }
        }
        }
        }
        }
        }
    }
  if(Time != 27000000)
  {
    ReadyToCalibration();
  }
}
void EndCalibration()
{
 ledcSetup(4, 1500, 8); //256
 ledcAttachPin(27, 4);
 ledcWrite(4, 200);
 delay(1000);
 ledcWrite(4, 0);
}

void Flag() {
  //Serial2.begin(115200);
 // for(int i =1; i <=8; i++ )
 // {
 //   Serial2.write('1');
 // }
  
}
*/