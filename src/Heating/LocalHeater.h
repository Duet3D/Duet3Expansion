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

class CanMessageHeaterTuningReport;

class LocalHeater : public Heater
{
	static const size_t NumPreviousTemperatures = 4; // How many samples we average the temperature derivative over

public:
	LocalHeater(unsigned int heaterNum);
	~LocalHeater();

	GCodeResult ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, const StringRef& reply) override;
	GCodeResult SetPwmFrequency(PwmFrequency freq, const StringRef& reply) override;
	GCodeResult ReportDetails(const StringRef& reply) const override;

	void Spin() override;									// Called in a tight loop to keep things running
	void SwitchOff() override;						// Not even standby - all heater power off
	void ResetFault() override;						// Reset a fault condition - only call this if you know what you are doing
	float GetTemperature() const override;			// Get the current temperature
	float GetAveragePWM() const override;			// Return the running average PWM to the heater. Answer is a fraction in [0, 1].
	float GetAccumulator() const override;			// Return the integral accumulator
	void Suspend(bool sus) override;				// Suspend the heater to conserve power or while doing Z probing
	GCodeResult TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) override;
	GCodeResult FeedForwardAdjustment(float fanPwmChange, float extrusionChange) noexcept override;

	static bool GetTuningCycleData(CanMessageHeaterTuningReport& msg);	// get a heater tuning cycle report, if we have one

protected:
	void ResetHeater() noexcept override;
	HeaterMode GetMode() const noexcept override { return mode; }
	GCodeResult SwitchOn(const StringRef& reply) noexcept override;		// Turn the heater on and set the mode
	GCodeResult UpdateModel(const StringRef& reply) noexcept override;	// Called when the heater model has been changed

private:
	void SetHeater(float power) const;				// Power is a fraction in [0,1]
	TemperatureError ReadTemperature();				// Read and store the temperature of this heater
	void DoTuningStep();							// Called on each temperature sample when auto tuning
	float GetExpectedHeatingRate() const;			// Get the minimum heating rate we expect
	void RaiseHeaterFault(HeaterFaultType type, const char *format, ...) noexcept;

	PwmPort ports[MaxPortsPerHeater];				// The port(s) that drive the heater
	float temperature;								// The current temperature
	float previousTemperatures[NumPreviousTemperatures]; // The temperatures of the previous NumDerivativeSamples measurements, used for calculating the derivative
	size_t previousTemperatureIndex;				// Which slot in previousTemperature we fill in next
	float iAccumulator;								// The integral LocalHeater component
	float lastPwm;									// The last PWM value we output, before scaling by kS
	float averagePWM;								// The running average of the PWM, after scaling.
	float lastTemperatureValue;								// the last temperature we recorded while heating up
	uint32_t lastTemperatureMillis;							// when we recorded the last temperature
	uint32_t timeSetHeating;						// When we turned on the heater
	uint32_t lastSampleTime;						// Time when the temperature was last sampled by Spin()

	uint16_t heatingFaultCount;						// Count of questionable heating behaviours

	uint8_t previousTemperaturesGood;				// Bitmap indicating which previous temperature were good readings
	HeaterMode mode;								// Current state of the heater
	uint8_t badTemperatureCount;					// Count of sequential dud readings

	static_assert(sizeof(previousTemperaturesGood) * 8 >= NumPreviousTemperatures, "too few bits in previousTemperaturesGood");
};

#endif /* SRC_LOCALHEATER_H_ */
