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
class CanMessageSetHeaterTemperature;
class CanMessageHeaterModelNewNew;
class CanMessageSetHeaterMonitors;
class CanMessageHeaterTuningCommand;

class Heater
{
public:
	Heater(unsigned int num);
	virtual ~Heater();

	// Configuration methods
	virtual GCodeResult ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, const StringRef& reply) = 0;
	virtual GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) = 0;
	virtual GCodeResult ReportDetails(const StringRef& reply) const = 0;

	virtual float GetTemperature() const = 0;					// Get the current temperature
	virtual float GetAveragePWM() const = 0;					// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	virtual void ResetFault() = 0;								// Reset a fault condition - only call this if you know what you are doing
	virtual void SwitchOff() = 0;
	virtual void Spin() = 0;
	virtual void Suspend(bool sus) = 0;							// Suspend the heater to conserve power or while doing Z probing
	virtual float GetAccumulator() const = 0;					// get the inertial term accumulator
	virtual GCodeResult TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) = 0;
	virtual GCodeResult FeedForwardAdjustment(float fanPwmChange, float extrusionChange) = 0;

	GCodeResult SetTemperature(const CanMessageSetHeaterTemperature& msg, const StringRef& reply);

	unsigned int GetHeaterNumber() const { return heaterNumber; }

	void GetFaultDetectionParameters(float& pMaxTempExcursion, float& pMaxFaultTime) const
		{ pMaxTempExcursion = maxTempExcursion; pMaxFaultTime = maxHeatingFaultTime; }

	GCodeResult SetFaultDetectionParameters(float pMaxTempExcursion, float pMaxFaultTime);
	GCodeResult SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply);

	float GetHighestTemperatureLimit() const;					// Get the highest temperature limit
	float GetLowestTemperatureLimit() const;					// Get the lowest temperature limit
	void SetHeaterMonitoring(HeaterMonitor *h);

	const FopDt& GetModel() const { return model; }				// Get the process model
	GCodeResult SetModel(unsigned int heater, const CanMessageHeaterModelNewNew& msg, const StringRef& reply) noexcept;

	bool IsHeaterEnabled() const								// Is this heater enabled?
		{ return model.IsEnabled(); }

	bool IsTuning() const { return GetMode() >= HeaterMode::firstTuningMode; }
	uint8_t GetModeByte() const { return (uint8_t)GetMode(); }

protected:
	static constexpr float FeedForwardMultiplier = 1.3;			// how much we over-compensate feedforward to allow for heat reservoirs during tuning

	virtual void ResetHeater() noexcept = 0;
	virtual HeaterMode GetMode() const noexcept = 0;
	virtual void SwitchOn() noexcept = 0;
	virtual GCodeResult UpdateModel(const StringRef& reply) noexcept = 0;

	int GetSensorNumber() const noexcept { return sensorNumber; }
	void SetSensorNumber(int sn) noexcept { sensorNumber = sn; }
	float GetMaxTemperatureExcursion() const noexcept { return maxTempExcursion; }
	float GetMaxHeatingFaultTime() const noexcept { return maxHeatingFaultTime; }
	float GetTargetTemperature() const noexcept { return requestedTemperature; }

	void SetRawPidParameters(float p_kP, float p_recipTi, float p_tD) { model.SetRawPidParameters(p_kP, p_recipTi, p_tD); }

	HeaterMonitor monitors[MaxMonitorsPerHeater];	// embedding them in the Heater uses less memory than dynamic allocation

private:
	FopDt model;

	unsigned int heaterNumber;
	int sensorNumber;								// the sensor number used by this heater
	float requestedTemperature;						// The required temperature
	float maxTempExcursion;							// The maximum temperature excursion permitted while maintaining the setpoint
	float maxHeatingFaultTime;						// How long a heater fault is permitted to persist before a heater fault is raised
};

#endif /* SRC_HEATING_HEATER_H_ */
