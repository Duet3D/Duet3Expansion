/*
 * SpiTemperatureSensor.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_SPITEMPERATURESENSOR_H_
#define SRC_HEATING_SPITEMPERATURESENSOR_H_

#include "SensorWithPort.h"

#if SUPPORT_SPI_SENSORS

#include <Platform.h>

class SpiTemperatureSensor : public SensorWithPort
{
protected:
	SpiTemperatureSensor(unsigned int sensorNum, const char *name, SpiMode spiMode, uint32_t clockFrequency);
	bool ConfigurePort(const CanMessageGenericParser& parser, const StringRef& reply, bool& seen);
	void InitSpi();
	TemperatureError DoSpiTransaction(const uint8_t dataOut[], size_t nbytes, uint32_t& rslt) const
		pre(nbytes <= 8);

	SharedSpiClient device;
};

#endif

#endif /* SRC_HEATING_SPITEMPERATURESENSOR_H_ */
