#include <Arduino.h>
#include "Chebishev.h"
#define N 4
float NUM[5] {0.0001146463806597, 0.000343939141979, 0.000343939141979, 0.0001146463806597}; //Массивы запалняются коэффициентами, предварительно полученными в матлабе 
float DEN[5] {1, -2.864408316858, 2.748565383963, -0.8832398960598};
float GX1[N], GY1[N], GZ1[N], AX1[N], AY1[N], AZ1[N];
float GX2[N], GY2[N], GZ2[N], AX2[N], AY2[N], AZ2[N];
float ChebishevGX, ChebishevGY, ChebishevGZ, ChebishevAX, ChebishevAY, ChebishevAZ;
/// @brief 
/// @param IMUSignalGyroX 
float ChebishevMathGyroX(float IMUSignalGyroX) 
{
   long t;
if (micros()-t>=2000)
{
  t = micros();
  for(int i = N - 1; i > 0; i--) GX1[i] = GX1[i-1];
  GX1[0] = IMUSignalGyroX;
  float sum1 = 0;
  for(int i = 0; i < N; i++)
  {
    sum1 = sum1 + GX1[i] * NUM[i]; 
  }
  for(int i = N - 1; i > 0; i--) GX2[i] = GX2[i - 1];
  float sum2 = 0;
  for(int i = 1; i < N; i++)
  {
    sum2 = sum2 + GX2[i] * DEN[i];
  }
  ChebishevGX = sum1 - sum2;
  GX2[0] = ChebishevGX;
} 
return ChebishevGX;
}
/// @brief 
/// @param IMUSignalGyroY 
float ChebishevMathGyroY(float IMUSignalGyroY) 
{
  long t;
if (micros()-t>=2000)
{
  t = micros();
  for(int i = N - 1; i > 0; i--) GY1[i] = GY1[i-1];
  GY1[0] = IMUSignalGyroY;
  float sum1 = 0;
  for(int i = 0; i < N; i++)
  {
    sum1 = sum1 + GY1[i] * NUM[i]; 
  }
  for(int i = N - 1; i > 0; i--) GY2[i] = GY2[i - 1];
  float sum2 = 0;
  for(int i = 1; i < N; i++)
  {
    sum2 = sum2 + GY2[i] * DEN[i];
  }
  ChebishevGY = sum1 - sum2;
  GY2[0] = ChebishevGY;
} 
return ChebishevGY;
}
/// @brief 
/// @param IMUSignalGyroZ 
float ChebishevMathGyroZ(float IMUSignalGyroZ) 
{
  long t;
if (micros()-t>=2000)
{
  t = micros();
  for(int i = N - 1; i > 0; i--) GZ1[i] = GZ1[i-1];
  GZ1[0] = IMUSignalGyroZ;
  float sum1 = 0;
  for(int i = 0; i < N; i++)
  {
    sum1 = sum1 + GZ1[i] * NUM[i]; 
  }
  for(int i = N - 1; i > 0; i--) GZ2[i] = GZ2[i - 1];
  float sum2 = 0;
  for(int i = 1; i < N; i++)
  {
    sum2 = sum2 + GZ2[i] * DEN[i];
  }
  ChebishevGZ = sum1 - sum2;
  GZ2[0] = ChebishevGZ;
} 
return ChebishevGZ;
}
/// @brief 
/// @param IMUSignalAccelX 
float ChebishevMathAccelX(float IMUSignalAccelX) 
{
  long t;
if (micros()-t>=2000)
{
  t = micros();
  for(int i = N - 1; i > 0; i--) AX1[i] = AX1[i-1];
  AX1[0] = IMUSignalAccelX;
  float sum1 = 0;
  for(int i = 0; i < N; i++)
  {
    sum1 = sum1 + AX1[i] * NUM[i]; 
  }
  for(int i = N - 1; i > 0; i--) AX2[i] = AX2[i - 1];
  float sum2 = 0;
  for(int i = 1; i < N; i++)
  {
    sum2 = sum2 + AX2[i] * DEN[i];
  }
  ChebishevAX = sum1 - sum2;
  AX2[0] = ChebishevAX;
} 
return ChebishevAX;
}
/// @brief 
/// @param IMUSignalAccelY 
float ChebishevMathAccelY(float IMUSignalAccelY) 
{
  long t;
if (micros()-t>=2000)
{
  t = micros();
  for(int i = N - 1; i > 0; i--) AY1[i] = AY1[i-1];
  AY1[0] = IMUSignalAccelY;
  float sum1 = 0;
  for(int i = 0; i < N; i++)
  {
    sum1 = sum1 + AY1[i] * NUM[i]; 
  }
  for(int i = N - 1; i > 0; i--) AY2[i] = AY2[i - 1];
  float sum2 = 0;
  for(int i = 1; i < N; i++)
  {
    sum2 = sum2 + AY2[i] * DEN[i];
  }
  ChebishevAY = sum1 - sum2;
  AY2[0] = ChebishevAY;
} 
return ChebishevAY;
}
/// @brief 
/// @param IMUSignalAZ 
float ChebishevMathAccelZ(float IMUSignalAccelZ) 
{
  long t;
if (micros()-t>=2000)
{
  t = micros();
  for(int i = N - 1; i > 0; i--) AZ1[i] = AZ1[i-1];
  AZ1[0] = IMUSignalAccelZ;
  float sum1 = 0;
  for(int i = 0; i < N; i++)
  {
    sum1 = sum1 + AZ1[i] * NUM[i]; 
  }
  for(int i = N - 1; i > 0; i--) AZ2[i] = AZ2[i - 1];
  float sum2 = 0;
  for(int i = 1; i < N; i++)
  {
    sum2 = sum2 + AZ2[i] * DEN[i];
  }
  ChebishevAZ = sum1 - sum2;
  AZ2[0] = ChebishevAZ;
} 
return ChebishevAZ;
}