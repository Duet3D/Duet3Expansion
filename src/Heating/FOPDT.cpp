/*
 * FOPDT.cpp
 *
 *  Created on: 16 Aug 2016
 *      Author: David
 */

#include "FOPDT.h"
#include <CanMessageFormats.h>

// Set up sensible defaults here in case the user enables the heater without specifying values for all the parameters.
FopDt::FopDt() noexcept
{
	Reset();
}

// Check the model parameters are sensible, if they are then save them and return true. If not then write an error message to reply and return false.
bool FopDt::SetParameters(const CanMessageHeaterModelV3& msg, const StringRef& reply) noexcept
{
	// DC 2017-06-20: allow S down to 0.01 for one of our OEMs (use > 0.0099 because >= 0.01 doesn't work due to rounding error)
	const char *_ecv_array const err =
		  (msg.basicModel.heatingRate/msg.basicModel.basicCoolingRate < 0.1) ? "estimated temperature rise too small"					// minimum 10C temperature rise (same as with earlier heater model)
		: (msg.basicModel.fanCoolingRate < 0.0) ? "fan reduces cooling rate"
		: (msg.basicModel.coolingRateExponent < 1.0 || msg.basicModel.coolingRateExponent > 1.6) ? "cooling rate exponent out of range"
		: (msg.basicModel.deadTime <= 0.0099) ? "dead time too small"
		: (msg.basicModel.deadTime * msg.basicModel.basicCoolingRate > 50.0) ? "dead time too close to cooling time constant"			// dead time less than half the cooling time constant
		: (msg.maxPwm <= 0.0099 || msg.maxPwm > 1.0) ? "PWM out of range"
		: nullptr;
	if (err == nullptr)
	{
		basicModel = msg.basicModel;
		maxPwm = msg.maxPwm;
		inverted = msg.inverted;
		CalcPidConstants(100.0);
		enabled = true;
		return true;
	}
	reply.printf("bad model parameters: %s", err);
	return false;
}

void FopDt::Reset() noexcept
{
	SetDefaultModel(DefaultToolHeaterModel);	// set some values so that we don't report rubbish in the OM
	enabled = false;																// heater is disabled until the parameters are set
}

void FopDt::SetDefaultModel(const HeaterModel& model) noexcept
{
	basicModel = model;
	basicModel.zero = 0;

	maxPwm = 1.0;
	inverted =  false;
	CalcPidConstants(basicModel.typicalTemperature);
	enabled = true;
}

// Get the PID parameters as reported by M301
M301PidParameters FopDt::GetM301PidParameters(bool forLoadChange) const noexcept
{
	M301PidParameters rslt;
	const PidParameters& pp = GetPidParameters(forLoadChange);
	const float reportedKp = pp.kP * 255.0;
	rslt.kP = reportedKp;
	rslt.kI = pp.recipTi * reportedKp;
	rslt.kD = pp.tD * reportedKp;
	return rslt;
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
	// Calculate the cooling rate per degC at this temperature. We assume the fan is at 20% speed.
	const float temperatureRise = max<float>(targetTemperature - NormalAmbientTemperature, 1.0);		// avoid division by zero!
	const float averageCoolingRatePerDegC = basicModel.GetTotalCoolingRate(temperatureRise, 0.2)/temperatureRise;
	loadChangeParams.kP = 0.7/(basicModel.heatingRate * basicModel.deadTime);
	loadChangeParams.recipTi = powf(averageCoolingRatePerDegC, 0.25)/(1.14 * powf(basicModel.deadTime, 0.75));	// Ti = 1.14 * timeConstant^0.25 * deadTime^0.75 (Ho et al)
	loadChangeParams.tD = basicModel.deadTime * 0.7;

	setpointChangeParams.kP = 0.7/(basicModel.heatingRate * basicModel.deadTime);
	setpointChangeParams.recipTi = fastSqrtf(averageCoolingRatePerDegC/basicModel.deadTime);			// Ti = timeConstant^0.5 * deadTime^0.5
	setpointChangeParams.tD = basicModel.deadTime * 0.7;
}

// End
