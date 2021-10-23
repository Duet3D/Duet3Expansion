/*
 * RemoteSensor.h
 *
 *  Created on: 23 Jul 2019
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_REMOTESENSOR_H_
#define SRC_HEATING_SENSORS_REMOTESENSOR_H_

#include "TemperatureSensor.h"

struct CanTemperatureReport;

class RemoteSensor : public TemperatureSensor
{
public:
	RemoteSensor(unsigned int sensorNum, CanAddress pBoardAddress) noexcept;
	~RemoteSensor() { }

	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) override { return GCodeResult::error; }	// should never be called
	CanAddress GetBoardAddress() const noexcept override { return boardAddress; }
	void Poll() noexcept override { }				// nothing to do here because reception of CAN messages update the reading
	void UpdateRemoteTemperature(CanAddress src, const CanSensorReport& report) noexcept override;

private:
	CanAddress boardAddress;
};

#endif /* SRC_HEATING_SENSORS_REMOTESENSOR_H_ */
