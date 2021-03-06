/*
 * LinearAdcTemperatureSensor.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_LINEARADCTEMPERATURESENSOR_H_
#define SRC_HEATING_LINEARADCTEMPERATURESENSOR_H_

#include "SpiTemperatureSensor.h"

#if SUPPORT_SPI_SENSORS

class CurrentLoopTemperatureSensor : public SpiTemperatureSensor
{
public:
	CurrentLoopTemperatureSensor(unsigned int sensorNum);
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) override;

	static constexpr const char *TypeName = "currentloop";

	void Poll() override;

private:
	TemperatureError TryGetLinearAdcTemperature(float& t);
	void CalcDerivedParameters();

	// Configurable parameters
	float tempAt4mA, tempAt20mA;
	uint32_t chipChannel, isDifferential;

	// Derived parameters
	float minLinearAdcTemp, linearAdcDegCPerCount;

	static constexpr float DefaultTempAt4mA = 385.0;
	static constexpr float DefaultTempAt20mA = 1600.0;
	static constexpr uint32_t DefaultChipChannel = 0;
};

#endif

#endif /* SRC_HEATING_LINEARADCTEMPERATURESENSOR_H_ */
