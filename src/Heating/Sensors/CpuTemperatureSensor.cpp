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

void CpuTemperatureSensor::Poll()
{
	float minT, currentT, maxT;
	Platform::GetMcuTemperatures(minT, currentT, maxT);
	SetResult(currentT, TemperatureError::success);
}

#endif

// End
