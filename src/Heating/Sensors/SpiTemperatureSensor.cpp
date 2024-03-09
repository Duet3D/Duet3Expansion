/*
 * SpiTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "SpiTemperatureSensor.h"

#if SUPPORT_SPI_SENSORS

#include <Platform/Tasks.h>

SpiTemperatureSensor::SpiTemperatureSensor(unsigned int sensorNum, const char *name, SpiMode spiMode, uint32_t clockFreq)
	: SensorWithPort(sensorNum, name), device(Platform::GetSharedSpi(), clockFreq, spiMode, NoPin, false)
{
}

bool SpiTemperatureSensor::ConfigurePort(const CanMessageGenericParser& parser, const StringRef& reply, bool& seen)
{
	const bool ret = SensorWithPort::ConfigurePort(parser, reply, PinAccess::write1, seen);
	device.SetCsPin(port.GetPin());
	if (ret)
	{
		ConfigureCommonParameters(parser, seen);
	}
	return ret;
}

void SpiTemperatureSensor::InitSpi()
{
}

// Send and receive 1 to 8 bytes of data and return the result as a single 32-bit word
TemperatureError SpiTemperatureSensor::DoSpiTransaction(const uint8_t dataOut[], size_t nbytes, uint32_t& rslt) const
{
	if (!device.Select(10))
	{
		return TemperatureError::busBusy;
	}

	delayMicroseconds(1);
	uint8_t rawBytes[8];
	const bool ok = device.TransceivePacket(dataOut, rawBytes, nbytes);
	delayMicroseconds(1);

	device.Deselect();
	delayMicroseconds(1);

	if (!ok)
	{
		return TemperatureError::timeout;
	}

	rslt = rawBytes[0];
	for (size_t i = 1; i < nbytes; ++i)
	{
		rslt <<= 8;
		rslt |= rawBytes[i];
	}

	return TemperatureError::ok;
}

// Send and receive data
TemperatureError SpiTemperatureSensor::DoSpiTransaction(const uint8_t dataOut[], uint8_t dataIn[], size_t nbytes) const noexcept
{
	if (!device.Select(10))
	{
		return TemperatureError::busBusy;
	}

	delayMicroseconds(1);
	const bool ok = device.TransceivePacket(dataOut, dataIn, nbytes);
	delayMicroseconds(1);

	device.Deselect();
	delayMicroseconds(1);

	if (!ok)
	{
		return TemperatureError::timeout;
	}

	return TemperatureError::ok;
}

#endif

// End
