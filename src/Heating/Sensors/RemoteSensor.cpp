/*
 * RemoteSensor.cpp
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#include "RemoteSensor.h"

#include <CanMessageBuffer.h>
#include <CanMessageFormats.h>
#include <General/Portability.h>

constexpr uint32_t RemoteTemperatureTimeoutMillis = 1000;

RemoteSensor::RemoteSensor(unsigned int sensorNum, CanAddress pBoardAddress) noexcept
	: TemperatureSensor(sensorNum, "remote"), boardAddress(pBoardAddress)
{
}

void RemoteSensor::UpdateRemoteTemperature(CanAddress src, const CanSensorReport& report) noexcept
{
	if (src == boardAddress)
	{
		SetResult(report.GetTemperature(), (TemperatureError)report.errorCode);
	}
}

// End
