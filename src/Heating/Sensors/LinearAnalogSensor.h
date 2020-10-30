/*
 * LinearAnalogSensor.h
 *
 *  Created on: 16 Apr 2019
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_LINEARANALOGSENSOR_H_
#define SRC_HEATING_SENSORS_LINEARANALOGSENSOR_H_

#include "SensorWithPort.h"

#if SUPPORT_THERMISTORS

class LinearAnalogSensor : public SensorWithPort
{
public:
	LinearAnalogSensor(unsigned int sensorNum);

	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) override;

	static constexpr const char *TypeName = "linearanalog";

	void Poll() override;

private:
	void CalcDerivedParameters();

	// Configurable parameters
	unsigned int thermistorInputChannel;
	float lowTemp, highTemp;
	bool filtered;

	// Derived parameters
	int adcFilterChannel;
	float linearIncreasePerCount;

	static constexpr float DefaultLowTemp = 0.0;
	static constexpr float DefaultHighTemp = 100.0;
};

#endif	//SUPPORT_THERMISTORS

#endif /* SRC_HEATING_SENSORS_LINEARANALOGSENSOR_H_ */
