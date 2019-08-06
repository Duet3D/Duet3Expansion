/*
 * CpuTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "CpuTemperatureSensor.h"
#include "Platform.h"

#if HAS_CPU_TEMP_SENSOR

CpuTemperatureSensor::CpuTemperatureSensor(unsigned int sensorNum) : TemperatureSensor(sensorNum, "MCU embedded temperature sensor")
{
}

TemperatureError CpuTemperatureSensor::TryGetTemperature(float& t)
{
	float minT, maxT;
	Platform::GetMcuTemperatures(minT, t, maxT);
	return TemperatureError::success;
}

#endif

// End
