/*
 * Heater.cpp
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#include "Heater.h"
#include "Platform.h"
#include "Heat.h"
#include "HeaterProtection.h"
#include "Sensors/TemperatureSensor.h"

Heater::Heater(unsigned int num)
	: heaterNumber(num), sensorNumber(-1), requestedTemperature(0.0),
	  maxTempExcursion(DefaultMaxTempExcursion), maxHeatingFaultTime(DefaultMaxHeatingFaultTime),
	  heaterProtection(nullptr)
{
}

Heater::~Heater()
{
}

// Set the process model returning true if successful
GCodeResult Heater::SetModel(float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted, const StringRef& reply)
{
	const float temperatureLimit = GetHighestTemperatureLimit();
	const bool rslt = model.SetParameters(gain, tc, td, maxPwm, temperatureLimit, voltage, usePid, inverted);
	if (rslt)
	{
		if (model.IsEnabled())
		{
			const GCodeResult rslt = UpdateModel(reply);
			if (rslt != GCodeResult::ok)
			{
				return rslt;
			}
			const float predictedMaxTemp = gain + NormalAmbientTemperature;
			const float noWarnTemp = (temperatureLimit - NormalAmbientTemperature) * 1.5 + 50.0;		// allow 50% extra power plus enough for an extra 50C
			if (predictedMaxTemp > noWarnTemp)
			{
				reply.printf("heater %u appears to be over-powered. If left on at full power, its temperature is predicted to reach %dC.\n",
						GetHeaterNumber(), (int)predictedMaxTemp);
				return GCodeResult::warning;
			}
		}
		else
		{
			ResetHeater();
		}
		return GCodeResult::ok;
	}

	reply.copy("bad model parameters");
	return GCodeResult::error;
}

GCodeResult Heater::SetFaultDetectionParameters(float pMaxTempExcursion, float pMaxFaultTime)
{
	maxTempExcursion = pMaxTempExcursion;
	maxHeatingFaultTime = pMaxFaultTime;
	return GCodeResult::ok;
}

GCodeResult Heater::SetTemperature(const CanMessageSetHeaterTemperature& msg, const StringRef& reply)
{
	switch (msg.command)
	{
	case CanMessageSetHeaterTemperature::commandNone:
		requestedTemperature = msg.setPoint;
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperature::commandOff:
		requestedTemperature = msg.setPoint;
		SwitchOff();
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperature::commandOn:
		requestedTemperature = msg.setPoint;
		SwitchOn();
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperature::commandResetFault:
		requestedTemperature = msg.setPoint;
		ResetFault();
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperature::commandSuspend:
		Suspend(true);
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperature::commandUnsuspend:
		requestedTemperature = msg.setPoint;
		Suspend(false);
		return GCodeResult::ok;

	default:
		break;
	}

	reply.printf("Unknown command %u to heater %u", msg.command, heaterNumber);
	return GCodeResult::ok;
}

// Get the highest temperature limit
float Heater::GetHighestTemperatureLimit() const
{
	return Heat::GetHighestTemperatureLimit(GetHeaterNumber());
}

// Get the lowest temperature limit
float Heater::GetLowestTemperatureLimit() const
{
	return Heat::GetLowestTemperatureLimit(GetHeaterNumber());
}

void Heater::SetHeaterProtection(HeaterProtection *h)
{
	heaterProtection = h;
}

// Check heater protection elements and return true if everything is good
bool Heater::CheckProtection() const
{
	for (HeaterProtection *prot = heaterProtection; prot != nullptr; prot = prot->Next())
	{
		if (!prot->Check())
		{
			// Something is not right
			return false;
		}
	}
	return true;
}

bool Heater::CheckGood() const
{
	return GetMode() == HeaterMode::fault && CheckProtection();
}

// End
