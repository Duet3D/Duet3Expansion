/*
 * FOPDT.h
 *
 *  Created on: 16 Aug 2016
 *      Author: David
 *
 *  Class to represent the parameters of a first order process with dead time
 */

#ifndef SRC_HEATING_FOPDT_H_
#define SRC_HEATING_FOPDT_H_

#include <RepRapFirmware.h>
#include <HeaterModel.h>

class StringRef;
class CanMessageHeaterModelV3;

// This is how PID parameters are stored internally
struct PidParameters
{
	float kP;			// controller (not model) gain
	float recipTi;		// reciprocal of controller integral time
	float tD;			// controller differential time
};

// This is how PID parameters are given in M301 commands
struct M301PidParameters
{
	float kP;
	float kI;
	float kD;
};

class FopDt
{
public:
	FopDt() noexcept;

	void Reset() noexcept;
	bool SetParameters(const CanMessageHeaterModelV3& msg, const StringRef& reply) noexcept;
	void SetDefaultModel(const HeaterModel& model) noexcept;
	const HeaterModel& GetBasicModel() const noexcept { return basicModel; }

	// Stored parameters
	float GetDeadTime() const noexcept { return basicModel.deadTime; }
	float GetMaxPwm() const noexcept { return maxPwm; }
	bool UsePid() const noexcept { return basicModel.usePid; }
	bool IsInverted() const noexcept { return inverted; }
	bool IsEnabled() const noexcept { return enabled; }

	float EstimateRequiredPwm(float temperatureRise, float fanPwm, float actualVoltage, float filamentPwm) const noexcept;
	float GetExpectedHeatingRate(float temperatureRise, float fanPwm, float heaterPwm, float actualVoltage, float filamentPwm) const noexcept;
	float GetPwmCorrectionForFan(float temperatureRise, float oldFanPwm, float newFanPwm) const noexcept;
	void CalcPidConstants(float targetTemperature) noexcept;

	// Derived parameters
	M301PidParameters GetM301PidParameters(bool forLoadChange) const noexcept;

	const PidParameters& GetPidParameters(bool forLoadChange) const noexcept
	{
		return (forLoadChange) ? loadChangeParams : setpointChangeParams;
	}

private:
	static float EstimateMaxTemperatureRise(float hr, float cr, float cre) noexcept;

	HeaterModel basicModel;
	float maxPwm;
	bool enabled;
	bool inverted;

	PidParameters setpointChangeParams;		// parameters for handling changes in the setpoint
	PidParameters loadChangeParams;			// parameters for handling changes in the load
};

// Get an estimate of the expected heating rate at the specified temperature rise and PWM. The result may be negative.
inline float FopDt::GetExpectedHeatingRate(float temperatureRise, float fanPwm, float heaterPwm, float actualVoltage, float filamentPwm) const noexcept
{
	return basicModel.GetExpectedHeatingRate(temperatureRise, fanPwm, heaterPwm, actualVoltage, filamentPwm);
}

// Get an estimate of the heater PWM required to maintain a specified temperature
inline float FopDt::EstimateRequiredPwm(float temperatureRise, float fanPwm, float actualVoltage, float filamentPwm) const noexcept
{
	return basicModel.GetExpectedPwm(temperatureRise, fanPwm, actualVoltage, filamentPwm);
}

#endif /* SRC_HEATING_FOPDT_H_ */
