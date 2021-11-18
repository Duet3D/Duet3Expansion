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

#include <cstdint>
#include "GCodeResult.h"

class StringRef;
class CanMessageHeaterModelNewNew;

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

class FileStore;

class FopDt
{
public:
	FopDt();

	void Reset() noexcept;
	bool SetParameters(const CanMessageHeaterModelNewNew& msg, float temperatureLimit) noexcept;
	bool SetParameters(float phr, float pcrFanOff, float pcrFanOn, float pcrExponent, float pdt, float pMaxPwm, float temperatureLimit, float pVoltage, bool pUsePid, bool pInverted) noexcept;
	void SetDefaultToolParameters() noexcept;
	void SetDefaultBedOrChamberParameters() noexcept;

	// Stored parameters
	float GetHeatingRate() const noexcept { return heatingRate; }
	float GetCoolingRateChangeFanOn() const noexcept { return coolingRateChangeFanOn; }
	float GetDeadTime() const noexcept { return deadTime; }
	float GetMaxPwm() const noexcept { return maxPwm; }
	float EstimateRequiredPwm(float temperatureRise, float fanPwm) const noexcept;
	float GetCoolingRate(float temperatureRise, float fanPwm) const noexcept;
	float GetNetHeatingRate(float temperatureRise, float fanPwm, float heaterPwm) const noexcept;
	float CorrectPwm(float requiredPwm, float actualVoltage) const noexcept;
	void AppendM307Command(unsigned int heaterNumber, const StringRef& str) const noexcept;
	void AppendParameters(const StringRef& str) const noexcept;
	bool UsePid() const noexcept { return usePid; }
	bool IsInverted() const noexcept { return inverted; }
	bool IsEnabled() const noexcept { return enabled; }

	// Derived parameters
	bool ArePidParametersOverridden() const noexcept { return pidParametersOverridden; }
	M301PidParameters GetM301PidParameters(bool forLoadChange) const noexcept;
	void SetM301PidParameters(const M301PidParameters& params) noexcept;
	void SetRawPidParameters(float p_kP, float p_recipTi, float p_tD) noexcept;

	const PidParameters& GetPidParameters(bool forLoadChange) const noexcept
	{
		return (forLoadChange) ? loadChangeParams : setpointChangeParams;
	}

private:
	void CalcPidConstants() noexcept;

	float heatingRate;
	float coolingRateFanOff;
	float coolingRateChangeFanOn;
	float coolingRateExponent;
	float deadTime;
	float maxPwm;
	float standardVoltage;					// power voltage reading at which tuning was done, or 0 if unknown
	bool enabled;
	bool usePid;
	bool inverted;
	bool pidParametersOverridden;

	PidParameters setpointChangeParams;		// parameters for handling changes in the setpoint
	PidParameters loadChangeParams;			// parameters for handling changes in the load
};

#endif /* SRC_HEATING_FOPDT_H_ */
