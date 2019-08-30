/*
 * ThermocoupleSensor31856.h
 *
 *  Created on: 27 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_THERMOCOUPLESENSOR31856_H_
#define SRC_HEATING_SENSORS_THERMOCOUPLESENSOR31856_H_

#include "SpiTemperatureSensor.h"

#if SUPPORT_SPI_SENSORS

class ThermocoupleSensor31856 : public SpiTemperatureSensor
{
public:
	ThermocoupleSensor31856(unsigned int sensorNum);
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) override;

	static constexpr const char *TypeName = "thermocouplemax31856";

	void Poll() override;

private:
	TemperatureError TryInitThermocouple() const;

	uint8_t cr0;
	uint8_t thermocoupleType;
};

#endif

#endif /* SRC_HEATING_SENSORS_THERMOCOUPLESENSOR31856_H_ */
