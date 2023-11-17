/*
 * SensorWithPort.cpp
 *
 *  Created on: 18 Jul 2019
 *      Author: David
 */

#include "SensorWithPort.h"
#include "CanMessageGenericParser.h"
#include <Platform/Platform.h>

SensorWithPort::SensorWithPort(unsigned int sensorNum, const char *type)
	: TemperatureSensor(sensorNum, type)
{
}

SensorWithPort::~SensorWithPort()
{
	port.Release();
}

// Try to configure the port. Return true if the port is valid at the end, else return false and set the error message in 'reply'. Set 'seen' if we saw the P parameter.
bool SensorWithPort::ConfigurePort(const CanMessageGenericParser& parser, const StringRef& reply, PinAccess access, bool& seen)
{
	String<StringLength20> portName;
	if (parser.GetStringParam('P', portName.GetRef()))
	{
		seen = true;
		const bool ok = port.AssignPort(portName.c_str(), reply, PinUsedBy::sensor, access);
		if (ok && access == PinAccess::readAnalog)
		{
			// If it's a thermistor port then we need to initialise the averaging filter
			Platform::InitThermistorFilter(port);
		}
		return ok;
	}
	if (port.IsValid())
	{
		return true;
	}
	reply.copy("Missing port name parameter");
	return false;
}

// Copy the basic details to the reply buffer. This hides the version in the base class.
void SensorWithPort::CopyBasicDetails(const StringRef& reply) const
{
	reply.printf("type %s using pin ", GetSensorType());
	port.AppendPinName(reply);
	reply.catf(", reading %.1f, last error: %s", (double)GetStoredReading(), GetLastError().ToString());
}

// End
