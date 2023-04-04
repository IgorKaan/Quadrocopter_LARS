#include <Arduino.h>
#include <GoodMorningCopter.h>
extern float f3, f4, f5, f6, accelZ;
float OldGyroX, OldGyroY, OldGyroZ, VGyroX, VGyroY, VGyroZ, Time, OldTime;
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
   
    ledcSetup(4, 1000, 8); //256
    ledcAttachPin(27, 4);
    OldTime = millis();
    OldGyroX = f4;
    OldGyroY = f5;
    OldGyroZ = f6;
    Serial.println(OldGyroX);
    delay(10);
    Serial.println(f4);
    VGyroX = f4 - OldGyroX;
    VGyroY = f5 - OldGyroY;
    VGyroZ = f6 - OldGyroZ;
    Serial.println(VGyroX);
    Serial.println(VGyroY);
    Serial.println(VGyroZ);
    //delay(5000);
    Serial.println("Bad");
    Serial.println(abs(f3));
    while((abs(f4 - OldGyroX) < 0.01) && (abs(f5 - OldGyroY) < 0.01) && (abs(f6 - OldGyroZ) < 0.01)) // (-1 < f4 < 1)&&(-1 < f5 <  1)&&(-1 < f6 < 1))
    {
        OldGyroX = f4;
        delay(10);
        Time = millis() - OldTime;
       if(Time >= 3000)
       {
        Serial.println(f3);
        Serial.println("Hi");
        ledcWrite(4, 200);
        delay(1000);
        ledcWrite(4, 0);
        Time = 27000000;
        break;
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

