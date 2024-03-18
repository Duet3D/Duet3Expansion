/*
 * Heat.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_HEATING_HEAT_H_
#define SRC_HEATING_HEAT_H_

/**
 * The master class that controls all the heaters controlled by the expansion board
 */

#include <RepRapFirmware.h>
#include <Heating/LocalHeater.h>
#include <CanMessageFormats.h>
#include <RTOSIface/RTOSIface.h>

class TemperatureSensor;
class FopDt;

namespace Heat
{
	// Methods that don't relate to a particular heater
	[[noreturn]] void TaskLoop(void *);
	void Init();												// Set everything up
	void Exit();												// Shut everything down

	GCodeResult ConfigureHeater(const CanMessageGeneric& msg, const StringRef& reply);
	GCodeResult ProcessM308(const CanMessageGeneric& msg, const StringRef& reply);
	GCodeResult ProcessM307New(const CanMessageHeaterModelNewNew& msg, const StringRef& reply);
	GCodeResult SetFaultDetection(const CanMessageSetHeaterFaultDetectionParameters& msg, const StringRef& reply);
	GCodeResult SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply);

	void SwitchOffAll();										// Turn all heaters off
	void ResetFault(int heater);								// Reset a heater fault - only call this if you know what you are doing

	// Methods that relate to sensors
	float GetSensorTemperature(int sensorNum, TemperatureError& err) noexcept;	// Result is in degrees Celsius
	void ProcessRemoteSensorsReport(CanAddress src, const CanMessageSensorTemperatures& msg) noexcept;

	// Methods that relate to a particular heater
	GCodeResult SetTemperature(const CanMessageSetHeaterTemperature& msg, const StringRef& reply);
	GCodeResult TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply);
	GCodeResult FeedForward(const CanMessageHeaterFeedForward& msg, const StringRef& reply);

	float GetAveragePWM(size_t heater)							// Return the running average PWM to the heater as a fraction in [0, 1].
	pre(heater < NumTotalHeaters);

	bool IsHeaterEnabled(size_t heater)							// Is this heater enabled?
	pre(heater < NumTotalHeaters);

	int GetHeaterChannel(size_t heater);						// Return the channel used by a particular heater, or -1 if not configured
	bool SetHeaterChannel(size_t heater, int channel);			// Set the channel used by a heater, returning true if bad heater or channel number
	const char *GetHeaterSensorName(size_t heater);				// Get the name of the sensor for a heater, or nullptr if it hasn't been named

	void SuspendHeaters(bool sus);								// Suspend the heaters to conserve power

	ReadLockedPointer<TemperatureSensor> FindSensor(int sn);	// Get a pointer to the temperature sensor entry
	ReadLockedPointer<TemperatureSensor> FindSensorAtOrAbove(unsigned int sn);	// Get a pointer to the first temperature sensor with the specified or higher number

	inline bool IsBedOrChamberHeater(int heater) { return false; }

	void Diagnostics(const StringRef& reply);

	void NewDriverFault();
	void NewHeaterFault();
};

#endif /* SRC_HEATING_HEAT_H_ */
