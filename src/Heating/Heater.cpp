/*
 * Heater.cpp
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#include "Heater.h"
#include <Platform/Platform.h>
#include <CanMessageFormats.h>
#include "Heat.h"
#include "Sensors/TemperatureSensor.h"

Heater::Heater(unsigned int num) : heaterNumber(num)
{
	Heater::ResetHeater();
}

Heater::~Heater()
{
}

void Heater::ResetHeater() noexcept
{
	lastExtrusionPwmBoost = 0.0;
	extrusionTemperatureBoost = 0.0;
	lastFanPwm = 0.0;
}

void Heater::SwitchOff() noexcept
{
	lastExtrusionPwmBoost = 0.0;
	extrusionTemperatureBoost = 0.0;
}

GCodeResult Heater::SetFaultDetectionParameters(const CanMessageSetHeaterFaultDetectionParameters& msg, const StringRef& reply)
{
	maxTempExcursion = msg.maxTempExcursion;
#if SUPPORT_INDUCTIVE_HEATER
	maxHeatingFaultTime = (IsCustom()) ? min<float>(msg.maxFaultTime, CustomHeaterMaxFaultTime) : msg.maxFaultTime;
#else
	maxHeatingFaultTime = msg.maxFaultTime;
#endif
	if (msg.version35)
	{
		maxBadTemperatureCount = msg.maxBadTemperatureCount;
	}
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

// This function may be overridden in class LocalHeater if the heater characteristics are fixed
GCodeResult Heater::SetModel(const CanMessageHeaterModelV3& msg, const StringRef& reply) noexcept
{
	const bool rslt = model.SetParameters(msg, reply);
	if (rslt)
	{
		if (!model.IsEnabled())
		{
			ResetHeater();
		}
		return GCodeResult::ok;
	}

	return GCodeResult::error;
}

GCodeResult Heater::SetTemperature(const CanMessageSetHeaterTemperatureV1& msg, const StringRef& reply)
{
	switch (msg.command)
	{
	case CanMessageSetHeaterTemperatureV1::commandNone:
		requestedTemperature = msg.setPoint;
		model.CalcPidConstants(requestedTemperature);
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperatureV1::commandOff:
		function = (HeaterFunction)msg.function;
		requestedTemperature = msg.setPoint;
		model.CalcPidConstants(requestedTemperature);
		SwitchOff();
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperatureV1::commandOn:
		function = (HeaterFunction)msg.function;
		requestedTemperature = msg.setPoint;
		model.CalcPidConstants(requestedTemperature);
		return SwitchOn(reply);

	case CanMessageSetHeaterTemperatureV1::commandResetFault:
		function = (HeaterFunction)msg.function;
		requestedTemperature = msg.setPoint;
		model.CalcPidConstants(requestedTemperature);
		ResetFault();
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperatureV1::commandSuspend:
		function = (HeaterFunction)msg.function;
		Suspend(true);
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperatureV1::commandUnsuspend:
		function = (HeaterFunction)msg.function;
		requestedTemperature = msg.setPoint;
		model.CalcPidConstants(requestedTemperature);
		Suspend(false);
		return GCodeResult::ok;

	case CanMessageSetHeaterTemperatureV1::commandReset:
		ResetHeater();
		return GCodeResult::ok;

	default:
		break;
	}

	reply.printf("Unknown command %u to heater %u", msg.command, heaterNumber);
	return GCodeResult::ok;
}

float Heater::GetMaxPwmFaultTime() const noexcept
{
	return max<float>(DefaultToolHeaterPwmFaultTime, 5 * model.GetDeadTime());
}

float Heater::GetPwmFaultLevel() const noexcept
{
	return DefaultToolHeaterPwmFaultLevel;
}

// End
