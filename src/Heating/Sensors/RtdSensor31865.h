/*
 * RtdSensor31865.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_RTDSENSOR31865_H_
#define SRC_HEATING_RTDSENSOR31865_H_

#include "SpiTemperatureSensor.h"

#if SUPPORT_SPI_SENSORS

class RtdSensor31865 : public SpiTemperatureSensor
{
public:
	RtdSensor31865(unsigned int sensorNum);
	GCodeResult Configure(unsigned int heater, const CanMessageM305& msg, const StringRef& reply) override;
	void Init() override;

	static constexpr const char *TypeName = "rtdmax31865";

protected:
	TemperatureError TryGetTemperature(float& t) override;

private:
	TemperatureError TryInitRtd() const;

	uint16_t rref;				// reference resistor in ohms
	uint8_t cr0;
};

#endif

#endif /* SRC_HEATING_RTDSENSOR31865_H_ */
