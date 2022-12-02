/*
 * CpuTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "CpuTemperatureSensor.h"
#include <Platform/Platform.h>

#if HAS_CPU_TEMP_SENSOR

CpuTemperatureSensor::CpuTemperatureSensor(unsigned int sensorNum) : TemperatureSensor(sensorNum, "MCU embedded temperature sensor")
{
}

void CpuTemperatureSensor::Poll()
{
	SetResult(Platform::GetMcuTemperatures().current, TemperatureError::ok);
}

#endif

// End
