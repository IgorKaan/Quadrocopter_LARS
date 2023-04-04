#include <PT1.h>
float gxPT1, gyPT1, gzPT1, axPT1, ayPT1, azPT1;
float KPT = 0.9; 
float pt1FilterApplyGX(float input)
{
  gxPT1 = gxPT1 + KPT * (input - gxPT1);
  return gxPT1;
}

float pt1FilterApplyGY(float input)
{
  gyPT1 = gyPT1 + KPT * (input - gyPT1);
  return gyPT1;
}

float pt1FilterApplyGZ(float input)
{
  gzPT1 = gzPT1 + KPT * (input - gzPT1);
  return gzPT1;
}

float pt1FilterApplyAX(float input)
{
  axPT1 = axPT1 + KPT * (input - axPT1);
  return axPT1;
}

float pt1FilterApplyAY(float input)
{
  ayPT1 = ayPT1 + KPT * (input - ayPT1);
  return ayPT1;
}

float pt1FilterApplyAZ(float input)
{
  azPT1 = azPT1 + KPT * (input - azPT1);
  return azPT1;
}