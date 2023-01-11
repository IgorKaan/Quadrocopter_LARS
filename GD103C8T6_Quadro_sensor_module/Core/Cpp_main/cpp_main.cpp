#include <cpp_main.hpp>
#include "MadgwickAHRS.h"
#include "MS5611.h"

// множитель фильтра
#define BETA 0.22f

MS5611 ms5611;

Madgwick m_filter;

// переменные для данных с гироскопов, акселерометров
//float g_x, g_y, g_z, a_x, a_y, a_z;

// получаемые углы ориентации
float f_yaw, f_pitch, f_roll;

// переменная для хранения частоты выборок фильтра
float fps = 1000;


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

	void MadgwickFilterRun(float g_x, float g_y, float g_z, float a_x, float a_y, float a_z) {

	  m_filter.setKoeff(fps, BETA);
	  // обновляем входные данные в фильтр
	  m_filter.update(g_x, g_y, g_z, a_x, a_y, a_z);

	  // получение углов yaw, pitch и roll из фильтра
	  f_yaw =  m_filter.getYawDeg();
	  f_pitch = m_filter.getPitchDeg();
	  f_roll = m_filter.getRollDeg();

	}

}
