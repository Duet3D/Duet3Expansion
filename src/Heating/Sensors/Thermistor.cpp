/*
 * Thermistor.cpp
 * Reads temperature from a thermistor or a PT1000 sensor connected to a thermistor port
 *
 *  Created on: 10 Nov 2016
 *      Author: David
 */

#include "Thermistor.h"

#if SUPPORT_THERMISTORS

#include <Platform/Platform.h>
#include <CanMessageGenericParser.h>

#if HAS_VREF_MONITOR
# include <Hardware/NonVolatileMemory.h>
#endif

// The Steinhart-Hart equation for thermistor resistance is:
// 1/T = A + B ln(R) + C [ln(R)]^3
//
// The simplified (beta) equation assumes C=0 and is:
// 1/T = A + (1/Beta) ln(R)
//
// The parameters that can be configured in RRF are R25 (the resistance at 25C), Beta, and optionally C.

// Create an instance with default values
Thermistor::Thermistor(unsigned int sensorNum, bool p_isPT1000)
	: SensorWithPort(sensorNum, (p_isPT1000) ? "PT1000" : "Thermistor"), adcFilterChannel(-1),
#if defined(M23CL)
	  r25(DefaultThermistorR25_M23CL), beta(DefaultThermistorBeta_M23CL), shC(DefaultThermistorC_M23CL),
#elif defined(SZP)
	  r25(DefaultThermistorR25_SZP), beta(DefaultThermistorBeta_SZP), shC(DefaultThermistorC_SZP),
#else
	  r25(DefaultThermistorR25), beta(DefaultThermistorBeta), shC(DefaultThermistorC),
#endif
	  seriesR(DefaultThermistorSeriesR),
	  isPT1000(p_isPT1000), adcLowOffset(0), adcHighOffset(0)
{
	CalcDerivedParameters();
}

// Get the ADC reading
int32_t Thermistor::GetRawReading(bool& valid) const noexcept
{
	if (adcFilterChannel >= 0)
	{
		// Filtered ADC channel
		const volatile ThermistorAveragingFilter * const tempFilter = Platform::GetAdcFilter(adcFilterChannel);
		valid = tempFilter->IsValid();
		return tempFilter->GetSum()/(tempFilter->NumAveraged() >> AdcOversampleBits);
	}

	// Raw ADC channel
	valid = true;
	return (uint32_t)port.ReadAnalog() << AdcOversampleBits;
}

// Configure the temperature sensor
GCodeResult Thermistor::Configure(const CanMessageGenericParser& parser, const StringRef& reply)
{
	bool changed = false;
	if (!ConfigurePort(parser, reply, PinAccess::readAnalog, changed))
	{
		return GCodeResult::error;
	}

	if (changed)
	{
		// We changed the port, so clear the ADC corrections and set up the ADC filter if there is one
		adcLowOffset = adcHighOffset = 0;

		adcFilterChannel = Platform::GetAveragingFilterIndex(port);
		if (adcFilterChannel >= 0)
		{
#ifdef TOOL1RR
			// The temp2 port is the thermistor on the LDC1612 sensor so change its default parameters
			if (adcFilterChannel == 2)
			{
				  r25 = DefaultThermistorR25_TOOL1RR_temp2;
				  beta = DefaultThermistorBeta_TOOL1RR_temp2;
				  shC = DefaultThermistorC_TOOL1RR_temp2;
			}
#endif
#if HAS_VREF_MONITOR
			// Default the H and L parameters to the values from nonvolatile memory
			NonVolatileMemory mem(NvmPage::common);
			adcLowOffset = mem.GetThermistorLowCalibration(adcFilterChannel);
			adcHighOffset = mem.GetThermistorHighCalibration(adcFilterChannel);
#endif
		}
	}

	changed = parser.GetFloatParam('R', seriesR) || changed;
	if (!isPT1000)
	{
		if (parser.GetFloatParam('B', beta))
		{
			shC = 0.0;						// if user changes B and doesn't define C, assume C=0
			changed = true;
		}
		changed = parser.GetFloatParam('C', shC) || changed;
		changed = parser.GetFloatParam('T', r25) || changed;
		if (changed)
		{
			CalcDerivedParameters();
		}
	}

	int16_t lVal;
	if (parser.GetIntParam('L', lVal))
	{
		if (lVal == 999)
		{
#if HAS_VREF_MONITOR
			const volatile ThermistorAveragingFilter *vssaFilter = Platform::GetVssaFilter(adcFilterChannel);			// this one may be null on SAMC21 tool boards
			if (vssaFilter == nullptr)
			{
				reply.copy("Thermistor input low-end auto calibration is not supported by this hardware");
				return GCodeResult::error;
			}
			else
			{
				bool valid;
				const int32_t val = GetRawReading(valid);
				if (valid)
				{
					const int32_t computedCorrection =
									(val - (int32_t)(vssaFilter->GetSum()/(ThermistorAveragingFilter::NumAveraged() >> AdcOversampleBits)))
										/(1 << (AnalogIn::AdcBits + AdcOversampleBits - 13));
					if (computedCorrection >= -127 && computedCorrection <= 127)
					{
						adcLowOffset = (int8_t)computedCorrection;
						reply.copy("Measured L correction for port \"");
						port.AppendPinName(reply);
						reply.catf("\" is %d", adcLowOffset);

						// Store the value in NVM
//						if (!reprap.GetGCodes().IsRunningConfigFile())
						{
							NonVolatileMemory mem(NvmPage::common);
							mem.SetThermistorLowCalibration(adcFilterChannel, adcLowOffset);
							mem.EnsureWritten();
						}
					}
					else
					{
						reply.copy("Computed correction is not valid. Check that you have placed a jumper across the thermistor input.");
						return GCodeResult::error;
					}
				}
				else
				{
					reply.copy("Temperature reading is not valid");
					return GCodeResult::error;
				}
			}
#else
			reply.copy("Thermistor input auto calibration is not supported by this hardware");
			return GCodeResult::error;
#endif
		}
		else
		{
			adcLowOffset = (int8_t)constrain<int>(lVal, std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max());
		}
		changed = true;
	}

	int16_t hVal;
	if (parser.GetIntParam('H', hVal))
	{
		if (hVal == 999)
		{
#if HAS_VREF_MONITOR
			bool valid;
			const int32_t val = GetRawReading(valid);
			if (valid)
			{
				const int32_t vrefReading = Platform::GetAdcFilter(VrefFilterIndex)->GetSum();
				const int32_t computedCorrection =
								(val - (int32_t)(vrefReading/(ThermistorAveragingFilter::NumAveraged() >> AdcOversampleBits)))
									/(1 << (AnalogIn::AdcBits + AdcOversampleBits - 13));
				if (computedCorrection >= -127 && computedCorrection <= 127)
				{
					adcHighOffset = (int8_t)computedCorrection;
					reply.copy("Measured H correction for port \"");
					port.AppendPinName(reply);
					reply.catf("\" is %d", adcHighOffset);

					// Store the value in NVM
					NonVolatileMemory mem(NvmPage::common);
					mem.SetThermistorHighCalibration(adcFilterChannel, adcHighOffset);
					mem.EnsureWritten();
				}
				else
				{
					reply.copy("Computed correction is not valid. Check that you have disconnected the thermistor.");
					return GCodeResult::error;
				}
			}
			else
			{
				reply.copy("Temperature reading is not valid");
				return GCodeResult::error;
			}
#else
			reply.copy("Thermistor input auto calibration is not supported by this hardware");
			return GCodeResult::error;
#endif
		}
		else
		{
			adcHighOffset = (int8_t)constrain<int>(hVal, std::numeric_limits<int8_t>::min(), std::numeric_limits<int8_t>::max());
		}
		changed = true;
	}

	if (!changed)
	{
		CopyBasicDetails(reply);
		if (isPT1000)
		{
			// For a PT1000 sensor, only the series resistor is configurable
			reply.catf(", R:%.1f", (double)seriesR);
		}
		else
		{
			reply.catf(", T:%.1f B:%.1f C:%.2e R:%.1f", (double)r25, (double)beta, (double)shC, (double)seriesR);
		}
		reply.catf(" L:%d H:%d", adcLowOffset, adcHighOffset);
	}

	return GCodeResult::ok;
}

// Get the temperature
void Thermistor::Poll()
{
	bool tempFilterValid;
	const int32_t averagedTempReading = GetRawReading(tempFilterValid);

#if HAS_VREF_MONITOR
	// Use the actual VSSA and VREF values read by the ADC
	const volatile ThermistorAveragingFilter * const vrefFilter = Platform::GetVrefFilter(adcFilterChannel);
	const volatile ThermistorAveragingFilter * const vssaFilter = Platform::GetVssaFilter(adcFilterChannel);			// this one may be null on SAMC21 tool boards
	if (tempFilterValid && vrefFilter->IsValid() && (vssaFilter == nullptr || vssaFilter->IsValid()))
	{
		const int32_t rawAveragedVssaReading = (vssaFilter == nullptr) ? 0 : vssaFilter->GetSum()/(vssaFilter->NumAveraged() >> Thermistor::AdcOversampleBits);
		const int32_t rawAveragedVrefReading = vrefFilter->GetSum()/(vrefFilter->NumAveraged() >> Thermistor::AdcOversampleBits);
		const int32_t averagedVssaReading = rawAveragedVssaReading + (adcLowOffset * (1 << (AnalogIn::AdcBits - 12 + Thermistor::AdcOversampleBits - 1)));
		const int32_t averagedVrefReading = rawAveragedVrefReading + (adcHighOffset * (1 << (AnalogIn::AdcBits - 12 + Thermistor::AdcOversampleBits - 1)));

		// VREF is the measured voltage at VREF less the drop of a 15 ohm (EXP3HC) or 10 ohm (TOOL1LC) or 27 ohm (EXP1HCL) resistor.
		// VSSA is the voltage measured across the VSSA fuse, plus a 10R resistor on TOOL1LC 1.1 and 1.2.
		// So we assume the same maximum load and 18 ohms maximum resistance for the fuse.
		// Assume a maximum ADC reading offset of 100.
		constexpr int32_t maxDrop = (OversampledAdcRange * VrefTopResistor)/MinVrefLoadR + (100 << Thermistor::AdcOversampleBits);

# if SAME5x		// SAMC21 uses 3.3V to feed VRef but we don't have it available to use as a reference voltage, so we use 5V instead
		if (averagedVrefReading < OversampledAdcRange - maxDrop)
		{
			SetResult(TemperatureError::badVref);
		}
		else
# endif
			if (averagedVssaReading > maxDrop)
		{
			SetResult(TemperatureError::badVssa);
		}
		else
		{
#else
	if (tempFilterValid)
	{
		{
#endif
			// Calculate the resistance
#if HAS_VREF_MONITOR
			if (averagedVrefReading <= averagedTempReading)
			{
				SetResult((isPT1000) ? BadErrorTemperature : ABS_ZERO, TemperatureError::openCircuit);
			}
			else if (averagedTempReading <= averagedVssaReading)
			{
				SetResult(BadErrorTemperature, TemperatureError::shortCircuit);
			}
			else
			{
				const float resistance = seriesR * (float)(averagedTempReading - averagedVssaReading)/(float)(averagedVrefReading - averagedTempReading);
#else
			const int32_t averagedVrefReading = OversampledAdcRange + adcHighOffset;
			if (averagedVrefReading <= averagedTempReading)
			{
				SetResult((isPT1000) ? BadErrorTemperature : ABS_ZERO, TemperatureError::openCircuit);
			}
			else
			{
				const float denom = (float)(averagedVrefReading - averagedTempReading) - 0.5;
				const int32_t averagedVssaReading = adcLowOffset;
				float resistance = seriesR * ((float)(averagedTempReading - averagedVssaReading) + 0.5)/denom;
#endif
				if (isPT1000)
				{
					// We want 100 * the equivalent PT100 resistance, which is 10 * the actual PT1000 resistance
					const uint16_t ohmsx100 = (uint16_t)lrintf(constrain<float>(resistance * 10, 0.0, 65535.0));
					float t;
					const TemperatureError sts = GetPT100Temperature(t, ohmsx100);
					SetResult(t, sts);
				}
				else
				{
					// Else it's a thermistor
					const float logResistance = logf(resistance);
					const float recipT = shA + shB * logResistance + shC * logResistance * logResistance * logResistance;
					const float temp = (recipT > 0.0) ? (1.0/recipT) + ABS_ZERO : BadErrorTemperature;

					// It's hard to distinguish between an open circuit and a cold high-resistance thermistor.
					// So we treat a temperature below -5C as an open circuit, unless we are using a low-resistance thermistor. The E3D thermistor has a resistance of about 470k @ -5C.
					if (temp < MinimumConnectedTemperature && resistance > seriesR * 100)
					{
						// Assume thermistor is disconnected
						SetResult(ABS_ZERO, TemperatureError::openCircuit);
					}
					else
					{
						SetResult(temp, TemperatureError::ok);
					}
				}
			}
		}
	}
	else
	{
		// Filter is not ready yet
		SetResult(TemperatureError::notReady);
	}
}

// Calculate shA and shB from the other parameters
void Thermistor::CalcDerivedParameters()
{
	shB = 1.0/beta;
	const float lnR25 = logf(r25);
	shA = 1.0/(25.0 - ABS_ZERO) - shB * lnR25 - shC * lnR25 * lnR25 * lnR25;
}

#endif	//SUPPORT_THERMISTORS

// End
