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
#include <RTOSIface/RTOSIface.h>

class TemperatureSensor;
class FopDt;
struct CanMessageGeneric;
struct CanMessageHeaterModelV3;
struct CanMessageSetHeaterFaultDetectionParameters;
struct CanMessageSetHeaterMonitors;
struct CanMessageHeaterFeedForwardV1;
struct CanMessageSensorTemperatures;
struct CanMessageSetHeaterTemperatureV1;
struct CanMessageHeaterTuningCommand;
class CanMessageBuffer;

namespace Heat
{
	// Methods that don't relate to a particular heater
	[[noreturn]] void TaskLoop(void *) noexcept;
	void Init() noexcept;												// Set everything up
	void Exit() noexcept;												// Shut everything down

	GCodeResult ConfigureHeater(const CanMessageGeneric& msg, const StringRef& reply, uint8_t& extra) noexcept;
	GCodeResult ProcessM308(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult ProcessM307(const CanMessageHeaterModelV3& msg, const StringRef& reply) noexcept;
	GCodeResult SetFaultDetection(const CanMessageSetHeaterFaultDetectionParameters& msg, const StringRef& reply) noexcept;
	GCodeResult SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply) noexcept;

	void SetDefaultHeaterModel(CanMessageBuffer& buf) noexcept;			// set and report the default model for a heater

	void SwitchOffAll() noexcept;										// Turn all heaters off. Takes the heaters read lock, so NOT safe to call from an ISR.
	void SwitchOffAllLocalFromISR() noexcept;							// Turn all heaters off without taking any locks. Safe to call from an ISR.
	void ResetFault(int heater) noexcept;								// Reset a heater fault - only call this if you know what you are doing

	// Methods that relate to sensors
	float GetSensorTemperature(int sensorNum, TemperatureError& err) noexcept;	// Result is in degrees Celsius
	void ProcessRemoteSensorsReport(CanAddress src, const CanMessageSensorTemperatures& msg) noexcept;

	// Methods that relate to a particular heater
	GCodeResult SetTemperature(const CanMessageSetHeaterTemperatureV1& msg, const StringRef& reply) noexcept;
	GCodeResult TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept;
	GCodeResult FeedForward(const CanMessageHeaterFeedForwardV1& msg, const StringRef& reply) noexcept;

	float GetAveragePWM(size_t heater) noexcept							// Return the running average PWM to the heater as a fraction in [0, 1].
	pre(heater < NumTotalHeaters);

	bool IsHeaterEnabled(size_t heater)	 noexcept						// Is this heater enabled?
	pre(heater < NumTotalHeaters);

	int GetHeaterChannel(size_t heater) noexcept;						// Return the channel used by a particular heater, or -1 if not configured
	bool SetHeaterChannel(size_t heater, int channel) noexcept;			// Set the channel used by a heater, returning true if bad heater or channel number
	const char *GetHeaterSensorName(size_t heater) noexcept;			// Get the name of the sensor for a heater, or nullptr if it hasn't been named

	void SuspendHeaters(bool sus) noexcept;								// Suspend the heaters to conserve power

	ReadLockedPointer<TemperatureSensor> FindSensor(int sn) noexcept;	// Get a pointer to the temperature sensor entry
	ReadLockedPointer<TemperatureSensor> FindSensorAtOrAbove(unsigned int sn) noexcept;	// Get a pointer to the first temperature sensor with the specified or higher number

	inline bool IsBedOrChamberHeater(int heater) noexcept { return false; }

	void Diagnostics(const StringRef& reply) noexcept;

	void NewDriverFault() noexcept;
	void NewHeaterFault() noexcept;

	constexpr uint32_t NormalHeaterPollInterval = HeatSampleIntervalMillis / HEATER_POLL_RATE_MULTIPLIER;
};

#endif /* SRC_HEATING_HEAT_H_ */
