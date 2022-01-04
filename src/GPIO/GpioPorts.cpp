/*
 * GpioDevice.cpp - General purpose output device
 *
 *  Created on: 8 Jul 2019
 *      Author: David
 */

#include "GpioPorts.h"
#include <CAN/CanInterface.h>
#include <CanMessageGenericParser.h>
#include <CanMessageGenericTables.h>

static PwmPort ports[MaxGpOutPorts];

GCodeResult GpioPorts::HandleM950Gpio(const CanMessageGeneric &msg, const StringRef &reply)
{
	// Get and validate the port number
	CanMessageGenericParser parser(msg, M950GpioParams);
	uint16_t gpioNumber;
	if (!parser.GetUintParam('P', gpioNumber))
	{
		reply.copy("Missing port number parameter in M950Gpio message");
		return GCodeResult::error;
	}
	if (gpioNumber >= MaxGpOutPorts)
	{
		reply.printf("GPIO port number %u is too high for expansion board %u", gpioNumber, CanInterface::GetCanAddress());
		return GCodeResult::error;
	}

	bool isServo;
	if (!parser.GetBoolParam('S', isServo))
	{
		isServo = false;
	}

	PwmFrequency freq;
	const bool seenFreq = parser.GetUintParam('Q', freq);
	if (!seenFreq)
	{
		freq = (isServo) ? DefaultServoRefreshFrequency : DefaultPinWritePwmFreq;
	}

	PwmPort& port = ports[gpioNumber];
	String<StringLength50> pinName;
	if (parser.GetStringParam('C', pinName.GetRef()))
	{
		// Creating or destroying a port
		const bool ok = port.AssignPort(pinName.c_str(), reply, PinUsedBy::gpout, (isServo) ? PinAccess::servo : PinAccess::pwm);
		if (ok && port.IsValid())
		{
			port.SetFrequency(freq);
		}
		return (ok) ? GCodeResult::ok : GCodeResult::error;
	}
	else
	{
		// Changing frequency, or reporting on a port
		if (!port.IsValid())
		{
			reply.printf("Board %u does not have GPIO/servo port %u", CanInterface::GetCanAddress(), gpioNumber);
			return GCodeResult::error;
		}

		if (seenFreq)
		{
			port.SetFrequency(freq);
		}
		else
		{
			reply.printf("GPIO/servo port %u", gpioNumber);
			port.AppendFullDetails(reply);
		}
		return GCodeResult::ok;
	}
}

GCodeResult GpioPorts::HandleGpioWrite(const CanMessageWriteGpio &msg, const StringRef &reply)
{
	if (msg.portNumber >= MaxGpOutPorts)
	{
		reply.printf("GPIO port# %u is too high for this expansion board", msg.portNumber);
		return GCodeResult::error;
	}

	ports[msg.portNumber].WriteAnalog(msg.pwm);
	return GCodeResult::ok;
}

// End
