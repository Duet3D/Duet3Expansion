/*
 * TmcDriverTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "TmcDriverTemperatureSensor.h"
#include "Platform.h"

#if HAS_SMART_DRIVERS

TmcDriverTemperatureSensor::TmcDriverTemperatureSensor(unsigned int sensorNum)
	: TemperatureSensor(sensorNum, "TMC temperature warnings")
{
}

TemperatureError TmcDriverTemperatureSensor::TryGetTemperature(float& t)
{
	t = Platform::GetTmcDriversTemperature();
	return TemperatureError::success;
}

#endif

// End
