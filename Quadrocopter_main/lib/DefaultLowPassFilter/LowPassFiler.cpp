#include <LowPassFilter.h>
const float K = 0.5;
float OutputGyroX = 0, OutputGyroY = 0, OutputGyroZ = 0, OutputAccelX = 0, OutputAccelY = 0, OutputAccelZ = 0;

float LowPassMathGX(float SensorSignalGX) 
{
    OutputGyroX = OutputGyroX*(1-K) + SensorSignalGX*K;
    return OutputGyroX;
}
float LowPassMathGY(float SensorSignalGY) 
{
    OutputGyroY = OutputGyroY*(1-K) + SensorSignalGY*K;
    return OutputGyroY;
}
float LowPassMathGZ(float SensorSignalGZ) 
{
    OutputGyroZ = OutputGyroZ*(1-K) + SensorSignalGZ*K;
    return OutputGyroZ;
}
float LowPassMathAX(float SensorSignalAX) 
{
    OutputAccelX = OutputAccelX*(1-K) + SensorSignalAX*K;
    return OutputAccelX;
}
float LowPassMathAY(float SensorSignalAY) 
{
    OutputAccelY = OutputAccelY*(1-K) + SensorSignalAY*K;
    return OutputAccelY;
}
float LowPassMathAZ(float SensorSignalAZ) 
{
    OutputAccelZ = OutputAccelZ*(1-K) + SensorSignalAZ*K;
    return OutputAccelZ;
}