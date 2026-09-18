/*
 * BoardTemperatureSensor.h
 *
 *  Created on: 15 Sept 2026
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_BOARDTEMPERATURESENSOR_H_
#define SRC_HEATING_SENSORS_BOARDTEMPERATURESENSOR_H_

#include "TemperatureSensor.h"

#if HAS_BOARD_THERMISTOR

class BoardTemperatureSensor : public TemperatureSensor
{
public:
	BoardTemperatureSensor(unsigned int sensorNum) noexcept;

	static constexpr const char *TypeName = "boardtemp";

	void Poll() noexcept override;

private:
	static SensorTypeDescriptor typeDescriptor;
};

#endif

#endif /* SRC_HEATING_SENSORS_BOARDTEMPERATURESENSOR_H_ */
