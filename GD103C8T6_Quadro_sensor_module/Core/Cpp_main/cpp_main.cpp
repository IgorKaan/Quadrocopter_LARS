#include <cpp_main.hpp>
#include "MS5611.h"

MS5611 ms5611;


extern "C" {

	void MS5611Begin() {
		while(!ms5611.begin(MS5611_ULTRA_HIGH_RES))
		{
			HAL_Delay(500);
		}
	}

	uint32_t MS5611GetTemperature() {
		return ms5611.readRawTemperature();
	}

}
