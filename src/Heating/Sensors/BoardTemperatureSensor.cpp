/*
 * BoardTemperatureSensor.cpp
 *
 *  Created on: 15 Sept 2026
 *      Author: David
 */

#include "BoardTemperatureSensor.h"

#if HAS_BOARD_THERMISTOR

#include <Platform/Platform.h>

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor BoardTemperatureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new BoardTemperatureSensor(sensorNum); } );

BoardTemperatureSensor::BoardTemperatureSensor(unsigned int sensorNum) noexcept : TemperatureSensor(sensorNum, "board temperature sensor")
{
}

void BoardTemperatureSensor::Poll() noexcept
{
	const auto temp = Platform::GetBoardTemperatureAndResult();
	SetResult(temp.first, temp.second);
}

#endif

// End
