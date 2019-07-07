/*
 * TmcDriverTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "TmcDriverTemperatureSensor.h"
#include "Platform.h"

#if HAS_SMART_DRIVERS

TmcDriverTemperatureSensor::TmcDriverTemperatureSensor(unsigned int channel) : TemperatureSensor(channel)
{
}

void TmcDriverTemperatureSensor::Init()
{
}

TemperatureError TmcDriverTemperatureSensor::TryGetTemperature(float& t)
{
	t = Platform::GetTmcDriversTemperature(GetSensorChannel() - FirstTmcDriversSenseChannel);
	return TemperatureError::success;
}

#endif

// End
