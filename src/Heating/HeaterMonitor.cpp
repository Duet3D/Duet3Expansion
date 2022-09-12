/*
 * HeaterProtection.cpp
 *
 *  Created on: 16 Nov 2017
 *      Author: Christian
 */

#include "HeaterMonitor.h"

#include <Platform/Platform.h>
#include "Heat.h"

HeaterMonitor::HeaterMonitor() noexcept
	: sensorNumber(-1), trigger(HeaterMonitorTrigger::Disabled), badTemperatureCount(0)
{
}

// Check if any action needs to be taken. Returns true if everything is OK
bool HeaterMonitor::Check() noexcept
{
	if (sensorNumber >= 0 && trigger != HeaterMonitorTrigger::Disabled)
	{
		TemperatureError err;
		const float temperature = Heat::GetSensorTemperature(sensorNumber, err);

		if (err != TemperatureError::success)
		{
			badTemperatureCount++;
			if (badTemperatureCount > MaxBadTemperatureCount)
			{
				debugPrintf("Temperature reading error on sensor %d\n", sensorNumber);
				return false;
			}
		}
		else
		{
			badTemperatureCount = 0;
			switch (trigger)
			{
			case HeaterMonitorTrigger::TemperatureExceeded:
				return (temperature <= limit);

			case HeaterMonitorTrigger::TemperatureTooLow:
				return (temperature >= limit);

			default:
				break;
			}
		}
	}
	return true;
}

// End
