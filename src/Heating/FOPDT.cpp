/*
 * FOPDT.cpp
 *
 *  Created on: 16 Aug 2016
 *      Author: David
 */

#include "FOPDT.h"
#include "CanMessageFormats.h"

// Set up sensible defaults here in case the user enables the heater without specifying values for all the parameters.
FopDt::FopDt()
{
	Reset();
}

bool FopDt::SetParameters(const CanMessageHeaterModelNewNew& msg, float temperatureLimit) noexcept
{
	// DC 2017-06-20: allow S down to 0.01 for one of our OEMs (use > 0.0099 because >= 0.01 doesn't work due to rounding error)
	const float maxTempIncrease = max<float>(1500.0, temperatureLimit + 500.0);
	if (   msg.heatingRate/msg.basicCoolingRate > 0.1								// minimum 10C temperature rise (same as with earlier heater model)
		&& EstimateMaxTemperatureRise(msg.heatingRate, msg.basicCoolingRate, msg.coolingRateExponent) <= maxTempIncrease
																					// max temperature increase within limits
		&& msg.fanCoolingRate >= 0.0
		&& msg.coolingRateExponent >= 1.0
		&& msg.coolingRateExponent <= 1.6
		&& msg.deadTime > 0.099
		&& msg.deadTime * msg.basicCoolingRate <= 50.0								// dead time less than half the cooling time constant
		&& msg.maxPwm > 0.0099
		&& msg.maxPwm <= 1.0
	   )
	{
		heatingRate = msg.heatingRate;
		basicCoolingRate = msg.basicCoolingRate;
		fanCoolingRate = msg.fanCoolingRate;
		coolingRateExponent = msg.coolingRateExponent;
		deadTime = msg.deadTime;
		maxPwm = msg.maxPwm;
		standardVoltage = msg.standardVoltage;
		usePid = msg.usePid;
		inverted = msg.inverted;
		pidParametersOverridden = msg.pidParametersOverridden;

		if (msg.pidParametersOverridden)
		{
			SetRawPidParameters(msg.kP, msg.recipTi, msg.tD);
		}
		else
		{
			CalcPidConstants(100.0);

		}
		enabled = true;
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
	heatingRate = DefaultToolHeaterHeatingRate;
	basicCoolingRate = DefaultToolHeaterBasicCoolingRate;
	deadTime = DefaultToolHeaterDeadTime;
	fanCoolingRate = 0.0;
	coolingRateExponent = DefaultToolHeaterCoolingRateExponent;
	maxPwm = 1.0;
	standardVoltage = 0.0;
	usePid = true;
	inverted = pidParametersOverridden = false;
	CalcPidConstants(200.0);
	enabled = true;
}

// Set up default parameters for a bed/chamber heater and enable the model
void FopDt::SetDefaultBedOrChamberParameters() noexcept
{
	heatingRate = DefaultBedHeaterHeatingRate;
	basicCoolingRate = DefaultBedHeaterBasicCoolingRate;
	deadTime = DefaultBedHeaterDeadTime;
	fanCoolingRate = 0.0;
	coolingRateExponent = DefaultBedHeaterCoolingRateExponent;
	maxPwm = 1.0;
	standardVoltage = 0.0;
	usePid = false;
	inverted = pidParametersOverridden = false;
	CalcPidConstants(60.0);
	enabled = true;
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

void FopDt::CalcPidConstants(float targetTemperature) noexcept
{
	if (!pidParametersOverridden)
	{
		// Calculate the cooling rate per degC at this temperature. We assume the fan is at 20% speed.
		const float temperatureRise = max<float>(targetTemperature - NormalAmbientTemperature, 1.0);		// avoid division by zero!
		const float averageCoolingRatePerDegC = GetCoolingRate(temperatureRise, 0.2)/temperatureRise;
		loadChangeParams.kP = 0.7/(heatingRate * deadTime);
		loadChangeParams.recipTi = powf(averageCoolingRatePerDegC, 0.25)/(1.14 * powf(deadTime, 0.75));		// Ti = 1.14 * timeConstant^0.25 * deadTime^0.75 (Ho et al)
		loadChangeParams.tD = deadTime * 0.7;

		setpointChangeParams.kP = 0.7/(heatingRate * deadTime);
		setpointChangeParams.recipTi = powf(averageCoolingRatePerDegC, 0.5)/powf(deadTime, 0.5);			// Ti = timeConstant^0.5 * deadTime^0.5
		setpointChangeParams.tD = deadTime * 0.7;
	}
}

// Adjust the actual heater PWM for supply voltage
float FopDt::CorrectPwmForVoltage(float requiredPwm, float actualVoltage) const noexcept
{
	if (requiredPwm < maxPwm && standardVoltage >= 10.0 && actualVoltage >= 10.0)
	{
		requiredPwm *= fsquare(standardVoltage/actualVoltage);
	}
	return min<float>(requiredPwm, maxPwm);
}

float FopDt::GetPwmCorrectionForFan(float temperatureRise, float fanPwmChange) const noexcept
{
	return temperatureRise * 0.01 * fanCoolingRate * fanPwmChange / heatingRate;
}

// Calculate the expected cooling rate for a given temperature rise above ambient
float FopDt::GetCoolingRate(float temperatureRise, float fanPwm) const noexcept
{
	temperatureRise *= 0.01;
	// If the temperature rise is negative then we must not try to raise it to a non-integral power!
	const float adjustedTemperatureRise = (temperatureRise < 0.0) ? -powf(-temperatureRise, coolingRateExponent) : powf(temperatureRise, coolingRateExponent);
	return basicCoolingRate * adjustedTemperatureRise + temperatureRise * fanCoolingRate * fanPwm;
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

/*static*/ float FopDt::EstimateMaxTemperatureRise(float hr, float cr, float cre) noexcept
{
	return 100.0 * powf(hr/cr, 1.0/cre);
}

// End
