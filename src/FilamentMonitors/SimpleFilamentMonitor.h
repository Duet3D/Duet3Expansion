/*
 * SimpleFilamentSensor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_SIMPLEFILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_SIMPLEFILAMENTMONITOR_H_

#include "FilamentMonitor.h"

class SimpleFilamentMonitor : public FilamentMonitor
{
public:
	SimpleFilamentMonitor(unsigned int extruder, unsigned int monitorType) noexcept;

	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept override;
	FilamentSensorStatus Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept override;
	FilamentSensorStatus Clear() noexcept override;
	void Diagnostics(const StringRef& reply) noexcept override;
	bool Interrupt() noexcept override;

private:
	void Poll() noexcept;

	bool highWhenNoFilament;
	bool filamentPresent;
	bool enabled;
};

#endif /* SRC_FILAMENTSENSORS_SIMPLEFILAMENTMONITOR_H_ */
