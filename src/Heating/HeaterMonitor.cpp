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
bool HeaterMonitor::Check(uint32_t maxBadTemperatureCount) noexcept
{
	if (sensorNumber >= 0 && trigger != HeaterMonitorTrigger::Disabled)
	{
		TemperatureError err(TemperatureError::unknownError);
		const float temperature = Heat::GetSensorTemperature(sensorNumber, err);

		if (err != TemperatureError::ok)
		{
			badTemperatureCount++;
			if (badTemperatureCount > maxBadTemperatureCount)
			{
#ifdef DEBUG
				debugPrintf("Temperature reading error on sensor %d\n", sensorNumber);
#endif
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
