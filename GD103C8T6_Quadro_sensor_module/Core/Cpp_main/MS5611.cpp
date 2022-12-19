#include "spi.h"
#include <math.h>
#include "main.h"
#include "MS5611.h"

uint8_t status = 5;

__weak void MS5611_OnActivate()
{
}

static inline void MS5611_Activate()
{
	MS5611_OnActivate();
	HAL_GPIO_WritePin(NSS_MS_GPIO_Port, NSS_MS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(NSS_W25Q_GPIO_Port, NSS_W25Q_Pin, GPIO_PIN_SET);
}

static inline void MS5611_Deactivate()
{
	HAL_GPIO_WritePin(NSS_MS_GPIO_Port, NSS_MS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(NSS_W25Q_GPIO_Port, NSS_W25Q_Pin, GPIO_PIN_SET);
}

uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&Byte, (uint8_t*)&receivedbyte, 1, 0x1000) != HAL_OK)
	{
		return -1;
	}
	else
	{
	}
	return receivedbyte;
}

void MS_SPI_Write (uint8_t *p_buffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	MS5611_Activate();
	SPIx_WriteRead(WriteAddr);
	while(NumByteToWrite>=0x01)
	{
		SPIx_WriteRead(*p_buffer);
		NumByteToWrite--;
		p_buffer++;
	}
	MS5611_Deactivate();
}

void MS_SPI_Read(uint8_t *p_buffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	MS5611_Activate();
	uint8_t data = ReadAddr;
	HAL_SPI_Transmit(&hspi2, &data, 1, HAL_MAX_DELAY);
	status = HAL_SPI_Receive(&hspi2, p_buffer, NumByteToRead, HAL_MAX_DELAY);
	if (HAL_SPI_Receive(&hspi2, p_buffer, NumByteToRead, HAL_MAX_DELAY) == HAL_OK) {
		//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	}
	else {
//		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
	MS5611_Deactivate();
}

void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
	MS_SPI_Read(dest, subAddress, count);
}

bool MS5611::begin(ms5611_osr_t osr)
{
    reset();

    setOversampling(osr);

    HAL_Delay(100);

    readPROM();

    return true;
}

// Set oversampling value
void MS5611::setOversampling(ms5611_osr_t osr)
{
    switch (osr)
    {
	case MS5611_ULTRA_LOW_POWER:
	    ct = 1;
	    break;
	case MS5611_LOW_POWER:
	    ct = 2;
	    break;
	case MS5611_STANDARD:
	    ct = 3;
	    break;
	case MS5611_HIGH_RES:
	    ct = 5;
	    break;
	case MS5611_ULTRA_HIGH_RES:
	    ct = 10;
	    break;
    }

    uosr = osr;
}

// Get oversampling value
ms5611_osr_t MS5611::getOversampling(void)
{
    return (ms5611_osr_t)uosr;
}

void MS5611::reset(void)
{
	MS5611_Activate();

	MS5611_Deactivate();
}

uint8_t MS5611::readPROM(void)
{

}

uint32_t MS5611::readRawTemperature(void)
{
    //Wire.beginTransmission(MS5611_ADDRESS);

    #if ARDUINO >= 100
	Wire.write(MS5611_CMD_CONV_D2 + uosr);
    #else
	//Wire.send(MS5611_CMD_CONV_D2 + uosr);
    #endif

    //Wire.endTransmission();
	SPIx_WriteRead(MS5611_CMD_CONV_D2 + uosr);
    HAL_Delay(ct);
    uint8_t temperature_temp[3];
    readRegisters(MS5611_CMD_ADC_READ, 3, temperature_temp);
    uint32_t temperature = temperature_temp[0] << 16 | temperature_temp[1] << 8 | temperature_temp[2];
    return temperature;
}

uint32_t MS5611::readRawPressure(void)
{
    //Wire.beginTransmission(MS5611_ADDRESS);

    #if ARDUINO >= 100
	Wire.write(MS5611_CMD_CONV_D1 + uosr);
    #else
	//Wire.send(MS5611_CMD_CONV_D1 + uosr);
    #endif

    //Wire.endTransmission();

    HAL_Delay(ct);

    return readRegister24(MS5611_CMD_ADC_READ);
}

int32_t MS5611::readPressure(bool compensation)
{
    uint32_t D1 = readRawPressure();

    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

    if (compensation)
    {
	int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

	OFF2 = 0;
	SENS2 = 0;

	if (TEMP < 2000)
	{
	    OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
	    SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
	}

	if (TEMP < -1500)
	{
	    OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
	    SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
	}

	OFF = OFF - OFF2;
	SENS = SENS - SENS2;
    }

    uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

double MS5611::readTemperature(bool compensation)
{
    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (compensation)
    {
	if (TEMP < 2000)
	{
	    TEMP2 = (dT * dT) / (2 << 30);
	}
    }

    TEMP = TEMP - TEMP2;

    return ((double)TEMP/100);
}

// Calculate altitude from Pressure & Sea level pressure
double MS5611::getAltitude(double pressure, double seaLevelPressure)
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)seaLevelPressure, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double MS5611::getSeaLevel(double pressure, double altitude)
{
    return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t MS5611::readRegister16(uint8_t reg)
{
    uint16_t value;
//    //Wire.beginTransmission(MS5611_ADDRESS);
//    #if ARDUINO >= 100
//        Wire.write(reg);
//    #else
//        //Wire.send(reg);
//    #endif
//    Wire.endTransmission();
//
//    Wire.beginTransmission(MS5611_ADDRESS);
//    Wire.requestFrom(MS5611_ADDRESS, 2);
//    while(!Wire.available()) {};
//    #if ARDUINO >= 100
//        uint8_t vha = Wire.read();
//        uint8_t vla = Wire.read();
//    #else
//        uint8_t vha = Wire.receive();
//        uint8_t vla = Wire.receive();
//    #endif;
//    Wire.endTransmission();
//
//    value = vha << 8 | vla;

    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t MS5611::readRegister24(uint8_t reg)
{
    uint32_t value;
//    //Wire.beginTransmission(MS5611_ADDRESS);
//    #if ARDUINO >= 100
//        Wire.write(reg);
//    #else
//        //Wire.send(reg);
//    #endif
//    //Wire.endTransmission();
//
//    //Wire.beginTransmission(MS5611_ADDRESS);
//    //Wire.requestFrom(MS5611_ADDRESS, 3);
//    while(!Wire.available()) {};
//    #if ARDUINO >= 100
//        uint8_t vxa = Wire.read();
//        uint8_t vha = Wire.read();
//        uint8_t vla = Wire.read();
//    #else
//        uint8_t vxa = Wire.receive();
//        uint8_t vha = Wire.receive();
//        uint8_t vla = Wire.receive();
//    #endif;
//    //Wire.endTransmission();
//
//    value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;

    return value;
}



