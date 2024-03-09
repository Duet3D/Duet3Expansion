/*
 * SensorWithPort.h
 *
 *  Created on: 18 Jul 2019
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_SENSORWITHPORT_H_
#define SRC_HEATING_SENSORS_SENSORWITHPORT_H_

#include "TemperatureSensor.h"

class SensorWithPort : public TemperatureSensor
{
protected:
	SensorWithPort(unsigned int sensorNum, const char *type);
	~SensorWithPort();

	// Try to configure the port
	bool ConfigurePort(const CanMessageGenericParser& parser, const StringRef& reply, PinAccess access, bool& seen);

	// Append the pin details to the reply buffer
	void AppendPinDetails(const StringRef& reply) const noexcept override;

	IoPort port;
};

#endif /* SRC_HEATING_SENSORS_SENSORWITHPORT_H_ */
