/*
 * Heater.cpp
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#include "Heater.h"
#include <Platform/Platform.h>
#include "Heat.h"
#include "Sensors/TemperatureSensor.h"

Heater::Heater(unsigned int num)
	: heaterNumber(num), sensorNumber(-1), requestedTemperature(0.0),
	  maxTempExcursion(DefaultMaxTempExcursion), maxHeatingFaultTime(DefaultMaxHeatingFaultTime), maxBadTemperatureCount(DefaultMaxBadTemperatureCount),
	  isBedOrChamber(false)
{
}

Heater::~Heater()
{
}

GCodeResult Heater::SetFaultDetectionParameters(const CanMessageSetHeaterFaultDetectionParameters& msg, const StringRef& reply)
{
	maxTempExcursion = msg.maxTempExcursion;
	maxHeatingFaultTime = msg.maxFaultTime;
	return GCodeResult::ok;
}

GCodeResult Heater::SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply)
{
	for (size_t i = 0; i < min<size_t>(msg.numMonitors, MaxMonitorsPerHeater); ++i)
	{
		monitors[i].Set(msg.monitors[i].sensor, msg.monitors[i].limit, (HeaterMonitorAction)msg.monitors[i].action, (HeaterMonitorTrigger)msg.monitors[i].trigger);
	}
	return GCodeResult::ok;
}

GCodeResult Heater::SetModel(unsigned int heater, const CanMessageHeaterModelNewNew& msg, const StringRef& reply) noexcept
{
	const float temperatureLimit = GetHighestTemperatureLimit();
	const bool rslt = model.SetParameters(msg, temperatureLimit);
	if (rslt)
	{
		if (model.IsEnabled())
		{
			return UpdateModel(reply);
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

GCodeResult Heater::SetTemperature(const CanMessageSetHeaterTemperature& msg, const StringRef& reply)
{
	switch (msg.command)
	{
	case CanMessageSetHeaterTemperature::commandNone:
		requestedTemperature = msg.setPoint;
		model.CalcPidConstants(requestedTemperature);
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperature::commandOff:
		isBedOrChamber = msg.isBedOrChamber;
		requestedTemperature = msg.setPoint;
		model.CalcPidConstants(requestedTemperature);
		SwitchOff();
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperature::commandOn:
		isBedOrChamber = msg.isBedOrChamber;
		requestedTemperature = msg.setPoint;
		model.CalcPidConstants(requestedTemperature);
		return SwitchOn(reply);

	case CanMessageSetHeaterTemperature::commandResetFault:
		isBedOrChamber = msg.isBedOrChamber;
		requestedTemperature = msg.setPoint;
		model.CalcPidConstants(requestedTemperature);
		ResetFault();
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperature::commandSuspend:
		isBedOrChamber = msg.isBedOrChamber;
		Suspend(true);
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperature::commandUnsuspend:
		isBedOrChamber = msg.isBedOrChamber;
		requestedTemperature = msg.setPoint;
		model.CalcPidConstants(requestedTemperature);
		Suspend(false);
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperature::commandReset:
		ResetHeater();
		return GCodeResult::ok;

	default:
		break;
	}

	reply.printf("Unknown command %u to heater %u", msg.command, heaterNumber);
	return GCodeResult::ok;
}

float Heater::GetHighestTemperatureLimit() const noexcept
{
	float limit = BadErrorTemperature;
	for (const HeaterMonitor& prot : monitors)
	{
		if (prot.GetTrigger() == HeaterMonitorTrigger::TemperatureExceeded)
		{
			const float t = prot.GetTemperatureLimit();
			if (limit == BadErrorTemperature || t > limit)
			{
				limit = t;
			}
		}
	}
	return limit;
}

float Heater::GetLowestTemperatureLimit() const noexcept
{
	float limit = ABS_ZERO;
	for (const HeaterMonitor& prot : monitors)
	{
		if (prot.GetTrigger() == HeaterMonitorTrigger::TemperatureTooLow)
		{
			const float t = prot.GetTemperatureLimit();
			if (limit == ABS_ZERO || t < limit)
			{
				limit = t;
			}
		}
	}
	return limit;
}

// End
