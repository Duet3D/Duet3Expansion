/*
 * FOPDT.cpp
 *
 *  Created on: 16 Aug 2016
 *      Author: David
 */

#include "FOPDT.h"
#include "CanMessageFormats.h"

// Heater 6 on the Duet 0.8.5 is disabled by default at startup so that we can use fan 2.
// Set up sensible defaults here in case the user enables the heater without specifying values for all the parameters.
FopDt::FopDt()
{
	Reset();
}

bool FopDt::SetParameters(const CanMessageHeaterModelNewNew& msg, float temperatureLimit) noexcept
{
	// DC 2017-06-20: allow S down to 0.01 for one of our OEMs (use > 0.0099 because >= 0.01 doesn't work due to rounding error)
	const float maxTempIncrease = max<float>(1500.0, temperatureLimit + 500.0);
	if (   msg.heatingRate/msg.coolingRate > 10.0									// minimum 10C temperature rise (same as with earlier heater model)
		&& msg.heatingRate/msg.coolingRate <= maxTempIncrease						// max temperature increase within limits
		&& msg.coolingRateChangeFanOn >= 0.0
		&& msg.coolingRateExponent >= 1.0
		&& msg.coolingRateExponent <= 1.6
		&& msg.deadTime > 0.099
		&& 0.5 >= msg.deadTime * (msg.coolingRate + msg.coolingRateChangeFanOn)		// dead time less then cooling time constant
		&& msg.maxPwm > 0.0099
		&& msg.maxPwm <= 1.0
	   )
	{
		heatingRate = msg.heatingRate;
		coolingRateFanOff = msg.coolingRate;
		coolingRateChangeFanOn = msg.coolingRateChangeFanOn;
		coolingRateExponent = msg.coolingRateExponent;
		deadTime = msg.deadTime;
		maxPwm = msg.maxPwm;
		standardVoltage = msg.standardVoltage;
		usePid = msg.usePid;
		inverted = msg.inverted;
		enabled = true;
		CalcPidConstants();

		if (msg.pidParametersOverridden)
		{
			SetRawPidParameters(msg.kP, msg.recipTi, msg.tD);
		}
		return true;
	}
	return false;
}

void FopDt::Reset() noexcept
{
	SetDefaultToolParameters();						// set some values so that we don't report rubbish in the OM
	enabled = false;								// heater is disabled until the parameters are set
}

// Set up default parameters for a tool heater and enable the model
void FopDt::SetDefaultToolParameters() noexcept
{
	heatingRate = DefaultHotEndHeaterHeatingRate;
	coolingRateFanOff = DefaultHotEndHeaterCoolingRate;
	deadTime = DefaultHotEndHeaterDeadTime;
	coolingRateChangeFanOn = 0.0;
	coolingRateExponent = DefaultHotEndHeaterCoolingRateExponent;
	maxPwm = 1.0;
	standardVoltage = 0.0;
	usePid = true;
	inverted = false;
	enabled = true;
	CalcPidConstants();
}

// Set up default parameters for a bed/chamber heater and enable the model
void FopDt::SetDefaultBedOrChamberParameters() noexcept
{
	heatingRate = DefaultBedHeaterHeatingRate;
	coolingRateFanOff = DefaultBedHeaterCoolingRate;
	deadTime = DefaultBedHeaterDeadTime;
	coolingRateChangeFanOn = 0.0;
	coolingRateExponent = DefaultBedHeaterCoolingRateExponent;
	maxPwm = 1.0;
	standardVoltage = 0.0;
	usePid = false;
	inverted = false;
	enabled = true;
	CalcPidConstants();
}

// Get the PID parameters as reported by M301
M301PidParameters FopDt::GetM301PidParameters(bool forLoadChange) const
{
	M301PidParameters rslt;
	const PidParameters& pp = GetPidParameters(forLoadChange);
	const float reportedKp = pp.kP * 255.0;
	rslt.kP = reportedKp;
	rslt.kI = pp.recipTi * reportedKp;
	rslt.kD = pp.tD * reportedKp;
	return rslt;
}

// Override the PID parameters. We set both sets to the same parameters.
void FopDt::SetM301PidParameters(const M301PidParameters& pp)
{
	SetRawPidParameters(pp.kP * (1.0/255.0), pp.kI/pp.kP, pp.kD/pp.kP);
}

void FopDt::SetRawPidParameters(float p_kP, float p_recipTi, float p_tD)
{
	loadChangeParams.kP = setpointChangeParams.kP = p_kP;
	loadChangeParams.recipTi = setpointChangeParams.recipTi = p_recipTi;
	loadChangeParams.tD = setpointChangeParams.tD = p_tD;
	pidParametersOverridden = true;
}

/* Re-calculate the PID parameters.
 * For some possible formulas, see "Comparison of some well-known PID tuning formulas", Computers and Chemical Engineering 30 (2006) 1416�1423,
 * available at http://www.ece.ualberta.ca/~marquez/journal_publications_files/papers/tan_cce_06.pdf
 * Here are some examples, where r = td/tc:
 *    Cohen-Coon (modified to use half the original Kc value):
 *     Kc = (0.67/G) * (r + 0.185)
 *     Ti = 2.5 * td * (tc + 0.185 * td)/(tc + 0.611 * td)
 *     Td = 0.37 * td * tc/(tc + 0.185 * td)
 *    Ho et al, best response to setpoint changes:
 *     Kc = (1.086/G) * (r^-0.869
 *     Ti = tc/(0.74 - 0.13 * r)
 *     Td = 0.348 * tc * r^0.914
 *    IAE-setpoint:
 *     Kc = (0.65/G) * r^-1.04432
 *     Ti = tc/(0.9895 + 0.09539 * r)
 *     Td = 0.50814 * tc * r^1.08433
 *    Ho et al, best response to load changes:
 *     Kc = (1.435/G) * r^-0.921
 *     Ti = 1.14 * tc * r^0.749
 *     Td = 0.482 * tc * r^1.137
 *    ITAE-load:
 *     Kc = (0.77902/G) * r^-1.06401
 *     Ti = (tc/1.14311) * r^0.70949
 *     Td = 0.57137 * tc * r^1.03826
 * However, none of these works well in this application. The setpoint-based methods have integral times comparable to the process time
 * constant. This makes them very slow to reach that target temperature. Typically, the power is reduced too soon, so the temperature
 * flattens out too soon it and then it takes a very long time for the integral term to accumulate to the required value. The load-based
 * ones tend to have massive overshoot when the setpoint is changed, and even in the steady state some of them have marginal stability.
 */

void FopDt::CalcPidConstants()
{
	const float averageCoolingRate = coolingRateFanOff + 0.5 * coolingRateChangeFanOn;
	loadChangeParams.kP = 0.7/(heatingRate * deadTime);
	loadChangeParams.recipTi = powf(averageCoolingRate, 0.25)/(1.14 * powf(deadTime, 0.75));	// Ti = 1.14 * timeConstant^0.25 * deadTime^0.75 (Ho et al)
	loadChangeParams.tD = deadTime * 0.7;

	setpointChangeParams.kP = 0.7/(heatingRate * deadTime);
	setpointChangeParams.recipTi = powf(coolingRateFanOff, 0.5)/powf(deadTime, 0.5);			// Ti = timeConstant^0.5 * deadTime^0.5
	setpointChangeParams.tD = deadTime * 0.7;

	pidParametersOverridden = false;
}

// Adjust the actual heater PWM for supply voltage
float FopDt::CorrectPwm(float requiredPwm, float actualVoltage) const noexcept
{
	if (requiredPwm < maxPwm && standardVoltage >= 10.0 && actualVoltage >= 10.0)
	{
		requiredPwm *= fsquare(standardVoltage/actualVoltage);
	}
	return max<float>(requiredPwm, maxPwm);
}

// Calculate the expected cooling rate for a given temperature rise abiie ambient
float FopDt::GetCoolingRate(float temperatureRise, float fanPwm) const noexcept
{
	return coolingRateFanOff * powf(temperatureRise, coolingRateExponent) + temperatureRise * coolingRateChangeFanOn;
}

// Get an estimate of the expected heating rate at the specified temperature rise and PWM. The result may be negative.
float FopDt::GetNetHeatingRate(float temperatureRise, float fanPwm, float heaterPwm) const noexcept
{
	return heatingRate * heaterPwm - GetCoolingRate(temperatureRise, fanPwm);
}

// Get an estimate of the heater PWM required to maintain a specified temperature
float FopDt::EstimateRequiredPwm(float temperatureRise, float fanPwm) const noexcept
{
	return GetCoolingRate(temperatureRise, fanPwm)/heatingRate;
}

// End
