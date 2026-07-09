/*
 * Heater.h
 *
 *  Created on: 24 Jul 2019
 *      Author: David
 */

#ifndef SRC_HEATING_HEATER_H_
#define SRC_HEATING_HEATER_H_

#include <RepRapFirmware.h>
#include "HeaterMonitor.h"
#include "FOPDT.h"

#include <CanId.h>

class HeaterMonitor;
class CanMessageGenericParser;
class CanMessageSetHeaterTemperatureV1;
class CanMessageHeaterModelV3;
class CanMessageSetHeaterMonitors;
class CanMessageHeaterTuningCommand;
class CanMessageSetHeaterFaultDetectionParameters;
class CanMessageHeaterFeedForwardV1;
class CanMessageBuffer;

class Heater
{
public:
	explicit Heater(unsigned int num) noexcept;
	virtual ~Heater();
	Heater(const Heater &_ecv_from) = delete;

	// Configuration methods
	virtual GCodeResult ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, int ambientSn, const StringRef& reply) noexcept = 0;
	virtual GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept = 0;
	virtual GCodeResult ReportDetails(const StringRef& reply) const noexcept = 0;

	virtual float GetTemperature() const noexcept = 0;					// Get the current temperature
	virtual float GetAveragePWM() const noexcept = 0;					// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	virtual void ResetFault() noexcept = 0;								// Reset a fault condition - only call this if you know what you are doing
	virtual void SwitchOff() noexcept;
	virtual void Spin() noexcept = 0;
	virtual void Suspend(bool sus) noexcept = 0;						// Suspend the heater to conserve power or while doing Z probing
	virtual float GetAccumulator() const noexcept = 0;					// get the inertial term accumulator
	virtual GCodeResult TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept = 0;
	virtual GCodeResult ApplyFeedForward(const CanMessageHeaterFeedForwardV1& msg, const StringRef& reply) noexcept = 0;
	virtual bool IsCustom() const noexcept = 0;							// returns true if this is a custom heater with unusual default model parameters
	virtual void SetDefaultHeaterModel(CanMessageBuffer& buf) noexcept = 0;	// set and return the default heater model

	GCodeResult SetTemperature(const CanMessageSetHeaterTemperatureV1& msg, const StringRef& reply) noexcept;

	unsigned int GetHeaterNumber() const noexcept { return heaterNumber; }

	GCodeResult SetFaultDetectionParameters(const CanMessageSetHeaterFaultDetectionParameters& msg, const StringRef& reply) noexcept;
	GCodeResult SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply) noexcept;

	void SetHeaterMonitoring(HeaterMonitor *h) noexcept;

	const FopDt& GetModel() const noexcept { return model; }			// Get the process model
	GCodeResult SetModel(const CanMessageHeaterModelV3& msg, const StringRef& reply) noexcept;

	bool IsHeaterEnabled() const noexcept								// Is this heater enabled?
		{ return model.IsEnabled(); }

	bool IsTuning() const noexcept { return GetMode() >= HeaterMode::firstTuningMode; }
	uint8_t GetModeByte() const noexcept { return (uint8_t)GetMode(); }

	HeaterFunction GetFunction() const noexcept { return function; }

protected:
	virtual void ResetHeater() noexcept;
	virtual HeaterMode GetMode() const noexcept = 0;
	virtual GCodeResult SwitchOn(const StringRef& reply) noexcept = 0;

	int GetSensorNumber() const noexcept { return sensorNumber; }
	int GetAmbientSensorNumber() const noexcept { return ambientSensorNumber; }
	void SetSensorNumber(int sn) noexcept { sensorNumber = sn; }
	void SetAmbientSensorNumber(int sn) noexcept { ambientSensorNumber = sn; }
	float GetMaxTemperatureExcursion() const noexcept { return maxTempExcursion; }
	float GetMaxHeatingFaultTime() const noexcept { return maxHeatingFaultTime; }
	float GetPwmFaultLevel() const noexcept;
	float GetMaxPwmFaultTime() const noexcept;
	uint32_t GetMaxBadTemperatureCount() const noexcept { return maxBadTemperatureCount; }
	float GetTargetTemperature() const noexcept { return requestedTemperature; }
	float GetHighestTemperatureLimit() const noexcept;

	HeaterMonitor monitors[MaxMonitorsPerHeater];						// embedding them in the Heater uses less memory than dynamic allocation
	volatile float lastFanPwm;											// The fan PWM when we last calculated heater feedforward for the fan
	volatile float lastExtrusionPwmBoost;								// The last value of extrusion boost we applied
	volatile float extrusionTemperatureBoost;							// the amount of extrusion temperature boost we are currently applying

	FopDt model;
	float maxHeatingFaultTime = DefaultMaxHeatingFaultTime;				// how long a heater fault is permitted to persist before a heater fault is raised

private:
	unsigned int heaterNumber;
	int sensorNumber = -1;												// the sensor number used by this heater
	int ambientSensorNumber = -1;										// the ambient sensor number used by this heater, if any
	float requestedTemperature = 0.0;									// the required temperature
	float maxTempExcursion = DefaultMaxTempExcursion;					// the maximum temperature excursion permitted while maintaining the setpoint
	uint32_t maxBadTemperatureCount = DefaultMaxBadTemperatureCount;	// the number of consecutive bad sensor readings we allow before raising a fault
	HeaterFunction function = HeaterFunction::tool;						// function of this heater when it was switched on
};

#endif /* SRC_HEATING_HEATER_H_ */
