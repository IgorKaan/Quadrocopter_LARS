#ifndef MS5611_h
#define MS5611_h

#include <main.h>

#define MS5611_ADDRESS                0x77

#define READWRITE_CMD 0x80
#define MULTIPLEBYTE_CMD 0x40
#define DUMMY_BYTE 0x00

#define MS5611_CMD_ADC_READ           0x00
#define MS5611_CMD_RESET              0x1E
#define MS5611_CMD_CONV_D1            0x40
#define MS5611_CMD_CONV_D2            0x50
#define MS5611_CMD_READ_PROM          0xA0  // A0?

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,
    MS5611_HIGH_RES         = 0x06,
    MS5611_STANDARD         = 0x04,
    MS5611_LOW_POWER        = 0x02,
    MS5611_ULTRA_LOW_POWER  = 0x00
} ms5611_osr_t;

class MS5611
{
    public:

	bool begin(ms5611_osr_t osr = MS5611_HIGH_RES);
	uint32_t readRawTemperature(void);
	uint32_t readRawPressure(void);
	double readTemperature(bool compensation = false);
	int32_t readPressure(bool compensation = false);
	double getAltitude(double pressure, double seaLevelPressure = 101325);
	double getSeaLevel(double pressure, double altitude);
	void setOversampling(ms5611_osr_t osr);
	ms5611_osr_t getOversampling(void);

    private:
	uint8_t fc_temp[14];
	uint16_t fc[6];
	uint8_t ct;
	uint8_t uosr;
	int32_t TEMP2;
	int64_t OFF2, SENS2;

	void reset(void);
	uint8_t readPROM(void);

	uint16_t readRegister16(uint8_t reg);
	uint32_t readRegister24(uint8_t reg);
};

#endif
