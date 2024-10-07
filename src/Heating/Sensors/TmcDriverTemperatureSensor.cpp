/*
 * TmcDriverTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "TmcDriverTemperatureSensor.h"
#include <Movement/Move.h>

#if HAS_SMART_DRIVERS

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor TmcDriverTemperatureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TmcDriverTemperatureSensor(sensorNum); } );

TmcDriverTemperatureSensor::TmcDriverTemperatureSensor(unsigned int sensorNum)
	: TemperatureSensor(sensorNum, "TMC temperature warnings")
{
}

void TmcDriverTemperatureSensor::Poll()
{
	SetResult(moveInstance->GetTmcDriversTemperature(), TemperatureError::ok);
}

#endif

// End
