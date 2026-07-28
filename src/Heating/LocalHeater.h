/*
 * Pid.h
 *
 *  Created on: 21 Jul 2016
 *      Author: David
 */

#ifndef SRC_LOCALHEATER_H_
#define SRC_LOCALHEATER_H_

/**
 * This class implements a PID controller for the heaters
 */

#include "Heater.h"
#include "FOPDT.h"
#include "TemperatureError.h"
#include "Hardware/IoPorts.h"

#define CHECK_HEATER_PWM			defined(TOOLINDX)			// we only monitor the heater PWM on TOOLINDX

class CanMessageHeaterTuningReport;

class LocalHeater : public Heater
{
	static const size_t NumPreviousTemperatures = 4; 			// How many samples we average the temperature derivative over

public:
	explicit LocalHeater(unsigned int heaterNum) noexcept;
	~LocalHeater();

	GCodeResult ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, int ambientSn, const StringRef& reply) noexcept override;
	GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept override;
	GCodeResult ReportDetails(const StringRef& reply) const noexcept override;

	void Spin() noexcept override;								// Called in a tight loop to keep things running
	void SwitchOff() noexcept override;							// Not even standby - all heater power off
	void ResetFault() noexcept override;						// Reset a fault condition - only call this if you know what you are doing
	float GetTemperature() const noexcept override;				// Get the current temperature
	float GetAveragePWM() const noexcept override;				// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	float GetAccumulator() const noexcept override;				// Return the integral accumulator
	void Suspend(bool sus) noexcept override;					// Suspend the heater to conserve power or while doing Z probing
	GCodeResult TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept override;
	GCodeResult ApplyFeedForward(const CanMessageHeaterFeedForwardV1& msg, const StringRef& reply) noexcept override;
#if SUPPORT_INDUCTIVE_HEATER
	bool IsInductiveHeater() const noexcept override;			// returns true if this is an inductive heater with special requirements
#endif
	void SetDefaultHeaterModel(CanMessageBuffer& buf) noexcept override;	// set and return the default heater model

	static bool GetTuningCycleData(CanMessageHeaterTuningReport& msg) noexcept;	// get a heater tuning cycle report, if we have one

protected:
	void ResetHeater() noexcept override;
	HeaterMode GetMode() const noexcept override { return mode; }
	GCodeResult SwitchOn(const StringRef& reply) noexcept override;		// Turn the heater on and set the mode

private:
	void SetHeater(float power) const noexcept;					// Power is a fraction in [0,1]
	TemperatureError ReadTemperature() noexcept;				// Read and store the temperature of this heater
	void DoTuningStep() noexcept;								// Called on each temperature sample when auto tuning
	float GetExpectedHeatingRate(float voltage) const noexcept;	 // Get the minimum heating rate we expect
	void RaiseHeaterFault(HeaterFaultType type, const char *format, ...) noexcept;
	void UpdateHeaterMode(float targetTemperature) noexcept;	// Determine and if necessary change the current heater mode
#if SUPPORT_LP5817
	void UpdateStatusLed() noexcept;
#endif
#if SUPPORT_INDUCTIVE_HEATER
	GCodeResult StartHeaterCalibration(const StringRef& reply) noexcept;
	GCodeResult CheckHeaterCalibrationComplete(const StringRef& reply) noexcept;
#endif

	PwmPort ports[MaxPortsPerHeater];							// The port(s) that drive the heater
	float temperature;											// The current temperature
	float ambientTemperature;									// the temperature of the ambient sensor
	float previousTemperatures[NumPreviousTemperatures];		// The temperatures of the previous NumDerivativeSamples measurements, used for calculating the derivative
	size_t previousTemperatureIndex;							// Which slot in previousTemperature we fill in next
	float iAccumulator;											// The integral LocalHeater component
	float lastPwm;												// The last PWM value we output, before scaling by kS
	float averagePWM;											// The running average of the PWM, after scaling.
	float lastTemperatureValue;									// the last temperature we recorded while heating up
	float lastExtrusionTemperatureBoost;						// the value of the feedforward temperature boost in the previous PID controller iteration
	uint32_t lastTemperatureMillis;								// when we recorded the last temperature
	uint32_t timeSetHeating;									// When we turned on the heater
	uint32_t lastSampleTime;									// Time when the temperature was last sampled by Spin()

	uint16_t heaterExcursionFaultCount;							// Count of questionable heater temperature excursions
#if CHECK_HEATER_PWM
	uint16_t heaterPwmFaultCount;								// Count of questionable PWM values
#endif

	uint8_t previousTemperaturesGood;							// Bitmap indicating which previous temperature were good readings
	HeaterMode mode;											// Current state of the heater
	uint8_t badTemperatureCount;								// Count of sequential dud readings

	static_assert(sizeof(previousTemperaturesGood) * 8 >= NumPreviousTemperatures, "too few bits in previousTemperaturesGood");
};

#endif /* SRC_LOCALHEATER_H_ */
