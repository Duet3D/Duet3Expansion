/*
 * SensorWithPort.cpp
 *
 *  Created on: 18 Jul 2019
 *      Author: David
 */

#include "SensorWithPort.h"
#include "CanMessageGenericParser.h"

SensorWithPort::SensorWithPort(unsigned int sensorNum, const char *type)
	: TemperatureSensor(sensorNum, type)
{
}

SensorWithPort::~SensorWithPort()
{
	port.Release();
}

void SensorWithPort::FlagForDeletion()
{
	port.Release();
	TemperatureSensor::FlagForDeletion();
}

// Try to configure the port. Return true if the port is valid at the end, else return false and set the error message in 'reply'. Set 'seen' if we saw the P parameter.
bool SensorWithPort::ConfigurePort(const CanMessageGenericParser& parser, const StringRef& reply, PinAccess access, bool& seen)
{
	String<StringLength20> portName;
	if (parser.GetStringParam('P', portName.GetRef()))
	{
		seen = true;
		return port.AssignPort(portName.c_str(), reply, PinUsedBy::sensor, access);
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
	reply.printf("Sensor %d", GetSensorNumber());
	reply.catf(" type %s using pin ", GetSensorType());
	port.AppendPinName(reply);
	reply.catf(", last error: %s", TemperatureErrorString(GetLastError()));
}

// End
