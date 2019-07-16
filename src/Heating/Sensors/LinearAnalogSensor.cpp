/*
 * LinearAnalogSensor.cpp
 *
 *  Created on: 16 Apr 2019
 *      Author: David
 */

#include "LinearAnalogSensor.h"
#include "Platform.h"
#include "CanMessageFormats.h"

LinearAnalogSensor::LinearAnalogSensor(unsigned int sensorNum)
	: TemperatureSensor(sensorNum), lowTemp(DefaultLowTemp), highTemp(DefaultHighTemp), filtered(true)
{
	CalcDerivedParameters();
}

GCodeResult LinearAnalogSensor::Configure(const CanMessageM305& msg, const StringRef& reply)
{
	M305_SET_IF_PRESENT(lowTemp, msg, L);
	M305_SET_IF_PRESENT(highTemp, msg, H);
	if (msg.GotParamF())
	{
		filtered = msg.paramF >= 1;
	}

	CalcDerivedParameters();
	return GCodeResult::ok;
}

void LinearAnalogSensor::Init()
{
	Platform::GetAdcFilter(thermistorInputChannel).Init(0);
}

TemperatureError LinearAnalogSensor::TryGetTemperature(float& t)
{
	const volatile ThermistorAveragingFilter& tempFilter = Platform::GetAdcFilter(thermistorInputChannel);
	int32_t tempReading;
	if (filtered)
	{
		if (!tempFilter.IsValid())
		{
			t = BadErrorTemperature;
			return TemperatureError::notReady;
		}
		tempReading = tempFilter.GetSum()/(tempFilter.NumAveraged() >> AdcOversampleBits);
	}
	else
	{
		tempReading = tempFilter.GetLastReading() << AdcOversampleBits;
	}

	t = (tempReading * linearIncreasePerCount) + lowTemp;
	return TemperatureError::success;
}

void LinearAnalogSensor::CalcDerivedParameters()
{
	linearIncreasePerCount = (highTemp - lowTemp)/((filtered) ? FilteredAdcRange : UnfilteredAdcRange);
}

// End
