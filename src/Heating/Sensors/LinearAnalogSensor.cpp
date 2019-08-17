/*
 * LinearAnalogSensor.cpp
 *
 *  Created on: 16 Apr 2019
 *      Author: David
 */

#include "LinearAnalogSensor.h"
#include "Platform.h"
#include "CanMessageGenericParser.h"

LinearAnalogSensor::LinearAnalogSensor(unsigned int sensorNum)
	: SensorWithPort(sensorNum, "Linear analog"), lowTemp(DefaultLowTemp), highTemp(DefaultHighTemp), filtered(true), adcFilterChannel(-1)
{
	CalcDerivedParameters();
}

GCodeResult LinearAnalogSensor::Configure(const CanMessageGenericParser& parser, const StringRef& reply)
{
	bool seen = false;
	if (!ConfigurePort(parser, reply, PinAccess::readAnalog, seen))
	{
		return GCodeResult::error;
	}

	if (parser.GetFloatParam('B', lowTemp))
	{
		seen = true;
	}
	if (parser.GetFloatParam('C', highTemp))
	{
		seen = true;
	}
	if (parser.GetBoolParam('F', filtered))
	{
		seen = true;
	}

	if (seen)
	{
		CalcDerivedParameters();
		if (adcFilterChannel >= 0)
		{
			Platform::GetAdcFilter(adcFilterChannel).Init(0);
		}
	}
	else
	{
		CopyBasicDetails(reply);
		reply.catf(", %sfiltered, range %.1f to %.1f", (filtered) ? "" : "un", (double)lowTemp, (double)highTemp);
	}
	return GCodeResult::ok;
}

void LinearAnalogSensor::Poll()
{
	const volatile ThermistorAveragingFilter& tempFilter = Platform::GetAdcFilter(thermistorInputChannel);
	int32_t tempReading;
	if (filtered)
	{
		if (!tempFilter.IsValid())
		{
			SetResult(TemperatureError::notReady);
		}
		tempReading = tempFilter.GetSum()/(tempFilter.NumAveraged() >> AdcOversampleBits);
	}
	else
	{
		tempReading = tempFilter.GetLastReading() << AdcOversampleBits;
	}

	SetResult((tempReading * linearIncreasePerCount) + lowTemp, TemperatureError::success);
}

void LinearAnalogSensor::CalcDerivedParameters()
{
	linearIncreasePerCount = (highTemp - lowTemp)/((filtered) ? FilteredAdcRange : UnfilteredAdcRange);
}

// End
