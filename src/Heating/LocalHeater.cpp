/*
 * Pid.cpp
 *
 *  Created on: 21 Jul 2016
 *      Author: David
 */

#include "LocalHeater.h"
#include "Heat.h"
#include <Platform/Platform.h>
#include <CanMessageBuffer.h>
#include <CanMessageGenericParser.h>
#include <CAN/CanInterface.h>
#include <General/SafeVsnprintf.h>

#if SUPPORT_LP5817
# include <Platform/LedStatusControl.h>
#endif

// Private constants
const uint32_t InitialTuningReadingInterval = 250;		// the initial reading interval in milliseconds
const uint32_t TempSettleTimeout = 20000;				// how long we allow the initial temperature to settle

// Variables used during heater tuning
static float tuningPwm;									// the PWM to use, 0..1
static float tuningHighTemp;							// the target upper temperature
static float tuningLowTemp;								// the target lower temperature
static float tuningPeakTempDrop;						// must be well below TuningHysteresis

static uint32_t dHigh;
static uint32_t dLow;
static uint32_t tOn;
static uint32_t tOff;
static float heatingRate;
static float coolingRate;
static uint32_t lastOffTime;
static uint32_t lastOnTime;
static float peakTemp;									// max or min temperature
static uint32_t peakTime;								// the time at which we recorded peakTemp
static float afterPeakTemp;								// temperature after max from which we start timing the cooling rate
static uint32_t afterPeakTime;							// the time at which we recorded afterPeakTemp
static float tuningVoltage;								// the VIN voltage with the heater on

static uint16_t cyclesDone;
static bool tuningCycleComplete;

// Member functions and constructors

LocalHeater::LocalHeater(unsigned int heaterNum) noexcept : Heater(heaterNum), mode(HeaterMode::off)
{
	LocalHeater::ResetHeater();
	SetHeater(0.0);							// set up the pin even if the heater is not enabled (for PCCB)

	// Time the sensor was last sampled.  During startup, we use the current
	// time as the initial value so as to not trigger an immediate warning from the Tick ISR.
	lastSampleTime = millis();
}

LocalHeater::~LocalHeater()
{
	LocalHeater::SwitchOff();
	for (auto& port : ports)
	{
		port.Release();
	}
}

// Returns true if this is a custom heater with unusual default model parameters
bool LocalHeater::IsCustom() const noexcept
{
#if SUPPORT_INDUCTIVE_HEATER
	return ports[0].IsInductiveHeaterPort();
#else
	return false;
#endif
}

// Set and return the default model for this heater
void LocalHeater::SetDefaultHeaterModel(CanMessageBuffer& buf) noexcept
{
	const CanRequestId rid = buf.msg.setDefaultHeaterModel.requestId;
	const CanAddress src = buf.id.Src();
#if SUPPORT_INDUCTIVE_HEATER
	if (ports[0].IsInductiveHeaterPort())
	{
		model.SetDefaultModel(InductiveHeaterDefaultModel);
	}
	else
#endif
	{
		switch ((HeaterFunction)buf.msg.setDefaultHeaterModel.heaterFunction)
		{
		default:
			{
				auto msg1 = buf.SetupResponseMessage<CanMessageStandardReply>(rid, CanInterface::GetCanAddress(), src);
				strcpy(msg1->text, "unknown heater function");
				msg1->resultCode = (uint32_t)GCodeResult::error;
			}
			return;

		case HeaterFunction::tool:
			model.SetDefaultModel(DefaultToolHeaterModel);
			break;

		case HeaterFunction::bed:
			model.SetDefaultModel(DefaultBedHeaterModel);
			break;

		case HeaterFunction::chamber:
			model.SetDefaultModel(DefaultChamberHeaterModel);
			break;
		}
	}
	auto msg2 = buf.SetupResponseMessage<CanMessageHeaterModelReport>(rid, CanInterface::GetCanAddress(), src);
	msg2->model = model.GetBasicModel();
	msg2->resultCode = (uint32_t)GCodeResult::ok;
}

float LocalHeater::GetTemperature() const noexcept
{
	return temperature;
}

float LocalHeater::GetAccumulator() const noexcept
{
	return iAccumulator;
}

void LocalHeater::SetHeater(float power) const noexcept
{
	for (auto& port : ports)
	{
		port.WriteAnalog(power);
	}
}

void LocalHeater::ResetHeater() noexcept
{
	Heater::ResetHeater();
	lastExtrusionTemperatureBoost = 0.0;
	mode = HeaterMode::off;
	previousTemperaturesGood = 0;
	previousTemperatureIndex = 0;
	iAccumulator = 0.0;
	badTemperatureCount = 0;
	averagePWM = lastPwm = 0.0;
	heaterExcursionFaultCount = 0;
#if CHECK_HEATER_PWM
	heaterPwmFaultCount = 0;
#endif
	temperature = BadErrorTemperature;
#if SUPPORT_LP5817
	UpdateStatusLed();
#endif
}

// Configure the heater port and the sensor number
GCodeResult LocalHeater::ConfigurePortAndSensor(const char *portName, PwmFrequency freq, unsigned int sn, int ambientSn, const StringRef& reply) noexcept
{
	if constexpr (MaxPortsPerHeater == 1)
	{
		if (!ports[0].AssignPort(portName, reply, PinUsedBy::heater, PinAccess::pwm))
		{
			return GCodeResult::error;
		}
#if SUPPORT_INDUCTIVE_HEATER
		if (ports[0].IsInductiveHeaterPort())
		{
			model.SetDefaultModel(InductiveHeaterDefaultModel);			// override the default model parameters
			maxHeatingFaultTime = CustomHeaterMaxFaultTime;
		}
#endif
	}
	else
	{
		PinAccess access[MaxPortsPerHeater];
		IoPort* portAddrs[MaxPortsPerHeater];
		for (size_t i = 0; i < MaxPortsPerHeater; ++i)
		{
			access[i] = PinAccess::pwm;
			portAddrs[i] = &ports[i];
		}
		if (IoPort::AssignPorts(portName, reply, PinUsedBy::heater, MaxPortsPerHeater, portAddrs, access) == 0)
		{
			return GCodeResult::error;
		}

	}

	for (auto& port : ports)
	{
		port.SetFrequency(freq);
	}

	SetSensorNumber(sn);
	SetAmbientSensorNumber(ambientSn);
	if (Heat::FindSensor(sn).IsNull())
	{
		reply.printf("Sensor number %u has not been defined", sn);
		return GCodeResult::warning;
	}
	if (ambientSn >= 0 && Heat::FindSensor(ambientSn).IsNull())
	{
		reply.printf("Sensor number %u has not been defined", ambientSn);
		return GCodeResult::warning;
	}

	if (monitors[0].GetTrigger() == HeaterMonitorTrigger::Disabled)
	{
		// Set up a default monitor
		monitors[0].Set(sn, DefaultHotEndTemperatureLimit, HeaterMonitorAction::GenerateFault, HeaterMonitorTrigger::TemperatureExceeded);
	}
	return GCodeResult::ok;
}

GCodeResult LocalHeater::SetPwmFrequency(PwmFrequency freq, const StringRef& reply) noexcept
{
	for (auto& port : ports)
	{
		port.SetFrequency(freq);
	}
	return GCodeResult::ok;
}

GCodeResult LocalHeater::ReportDetails(const StringRef& reply) const noexcept
{
	reply.printf("Heater %u pin(s) ", GetHeaterNumber());
	ports[0].AppendPinName(reply);
	if constexpr (MaxPortsPerHeater > 1)
	{
		for (size_t i = 1; i < MaxPortsPerHeater && ports[i].IsValid(); ++i)
		{
			reply.cat('+');
			ports[i].AppendPinName(reply, false);
		}
	}

	ports[0].AppendFrequency(reply);

	if (GetSensorNumber() >= 0)
	{
		reply.catf(", sensor %d", GetSensorNumber());
	}
	else
	{
		reply.cat(", no sensor");
	}
	if (GetAmbientSensorNumber() >= 0)
	{
		reply.catf(", ambient sensor %d", GetAmbientSensorNumber());
	}
	else
	{
		reply.cat(", no ambient sensor");
	}
	return GCodeResult::ok;
}

// Read and store the temperature of this heater and returns the error code.
TemperatureError LocalHeater::ReadTemperature() noexcept
{
	TemperatureError err(TemperatureError::unknownError);
	temperature = Heat::GetSensorTemperature(GetSensorNumber(), err);						// in the event of an error, err is set and BAD_ERROR_TEMPERATURE is returned
	if (GetAmbientSensorNumber() >= 0)
	{
		TemperatureError err2(TemperatureError::unknownError);
		ambientTemperature = Heat::GetSensorTemperature(GetAmbientSensorNumber(), err2);	// in the event of an error, err is set and BAD_ERROR_TEMPERATURE is returned
		if (err2 != TemperatureError::ok)
		{
			ambientTemperature = NormalAmbientTemperature;
		}
	}
	else
	{
		ambientTemperature = NormalAmbientTemperature;
	}
	return err;
}

// This must be called whenever the heater is turned on, and any time the heater is active and the target temperature is changed
GCodeResult LocalHeater::SwitchOn(const StringRef& reply) noexcept
{
	if (!GetModel().IsEnabled())
	{
		reply.printf("Failed to turn on heater %u because its model is disabled", GetHeaterNumber());
		return GCodeResult::error;
	}

	if (mode == HeaterMode::fault)
	{
		reply.printf("Failed to turn on heater %u because it is in a fault state", GetHeaterNumber());
		return GCodeResult::error;
	}

	const float target = min<float>(GetTargetTemperature() + extrusionTemperatureBoost, GetHighestTemperatureLimit());
	UpdateHeaterMode(target);
#if SUPPORT_LP5817
	UpdateStatusLed();
#endif
	return GCodeResult::ok;
}

// Determine and if necessary change the current heater mode
void LocalHeater::UpdateHeaterMode(float targetTemperature) noexcept
{
	const HeaterMode newMode = (temperature + TemperatureCloseEnough < targetTemperature) ? HeaterMode::heating
					: (temperature > targetTemperature + TemperatureCloseEnough) ? HeaterMode::cooling
						: HeaterMode::stable;
	if (newMode != mode)
	{
		// The Heat task can preempt the GCodes task that calls this, so lock out the Heat task while we update multiple variables
		TaskCriticalSectionLocker lock;
		if (newMode == HeaterMode::heating)
		{
			lastTemperatureValue = temperature;
			lastTemperatureMillis = timeSetHeating = millis();
		}
		heaterExcursionFaultCount = 0;
#if CHECK_HEATER_PWM
		heaterPwmFaultCount = 0;
#endif
		mode = newMode;
#if SUPPORT_LP5817
		UpdateStatusLed();
#endif
	}
}

// Switch off the specified heater. If in tuning mode, delete the array used to store tuning temperature readings.
void LocalHeater::SwitchOff() noexcept
{
	lastPwm = 0.0;
	if (GetModel().IsEnabled())
	{
		SetHeater(0.0);
		if (mode > HeaterMode::off)
		{
			mode = HeaterMode::off;
		}
	}
	Heater::SwitchOff();
	lastExtrusionTemperatureBoost = 0.0;
#if SUPPORT_LP5817
	UpdateStatusLed();
#endif
}

// This is the main heater control loop function
void LocalHeater::Spin() noexcept
{
	// Read the temperature even if the heater is suspended or the model is not enabled
	const TemperatureError err = ReadTemperature();

	// Handle any temperature reading error and calculate the temperature rate of change, if possible
	if (err != TemperatureError::ok)
	{
		previousTemperaturesGood <<= 1;				// this reading isn't a good one
		if (mode > HeaterMode::suspended)			// don't worry about errors when reading heaters that are switched off or flagged as having faults
		{
			// Error may be a temporary error and may correct itself after a few additional reads
			badTemperatureCount++;
			if (badTemperatureCount > GetMaxBadTemperatureCount())
			{
				RaiseHeaterFault(HeaterFaultType::failedToReadSensor, "%s", err.ToString());
			}
		}
		// We leave lastPWM alone if we have a temporary temperature reading error
	}
	else
	{
		// We have an apparently-good temperature reading. Calculate the derivative, if possible.
		float derivative = 0.0;
		bool gotDerivative = false;
		badTemperatureCount = 0;
		if ((previousTemperaturesGood & (1u << (NumPreviousTemperatures - 1))) != 0)
		{
			const float tentativeDerivative = ((float)SecondsToMillis/Heat::NormalHeaterPollInterval) * (temperature - previousTemperatures[previousTemperatureIndex])
							/ (float)(NumPreviousTemperatures);
			// Some sensors give occasional temperature spikes. We don't expect the temperature to increase by more than 10C/second.
			if (fabsf(tentativeDerivative) <= 10.0)
			{
				derivative = tentativeDerivative;
				gotDerivative = true;
			}
		}
		previousTemperatures[previousTemperatureIndex] = temperature;
		previousTemperaturesGood = (previousTemperaturesGood << 1) | 1u;
		previousTemperatureIndex = (previousTemperatureIndex + 1) % NumPreviousTemperatures;

		if (GetModel().IsEnabled())
		{
			// Get the target temperature and the error
			const float targetTemperature = min<float>(GetTargetTemperature() + extrusionTemperatureBoost, GetHighestTemperatureLimit());

			if (IsPidMode(mode) && extrusionTemperatureBoost != lastExtrusionTemperatureBoost)
			{
				UpdateHeaterMode(targetTemperature);									// calculate new heater mode to prevent heater fault due to exceededAllowedExcursion
				lastExtrusionTemperatureBoost = extrusionTemperatureBoost;
			}

			const float error = targetTemperature - temperature;
#if HAS_VOLTAGE_MONITOR
			const float currentVoltage = (GetFunction() == HeaterFunction::tool) ? Platform::GetCurrentVinVoltage() : 0.0;		// correct PWM for voltage if it is a tool heater
#else
			constexpr float currentVoltage = 0.0;
#endif
			// Do the heating checks
			switch (mode)
			{
			case HeaterMode::heating:
				if (error <= TemperatureCloseEnough)
				{
					mode = HeaterMode::stable;
					heaterExcursionFaultCount = 0;
#if CHECK_HEATER_PWM
					heaterPwmFaultCount = 0;
#endif
				}
				else
				{
					const uint32_t now = millis();
					if ((float)(now - timeSetHeating) < GetModel().GetDeadTime() * SecondsToMillis * 2)		// wait for twice the dead time before we start looking at the temperature rise
					{
						// Record the temperature for when we are past the dead time
						lastTemperatureValue = temperature;
						lastTemperatureMillis = now;
					}
					else if (gotDerivative)												// this is a check in case we just had a temperature spike
					{
						const float expectedRate = GetExpectedHeatingRate(currentVoltage);
						const float minSamplingInterval = 3.0/expectedRate;				// check the temperature if we expect a 3C rise since last time
						const float actualInterval = (float)(now - lastTemperatureMillis) * MillisToSeconds;
						if (actualInterval >= minSamplingInterval)
						{
							// Check that we are heating fast enough, and if so, take another sample
							const float expectedTemperatureRise = expectedRate * actualInterval;
							const float actualTemperatureRise = temperature - lastTemperatureValue;
							// Bed heaters sometimes have much slower long term heating rates than their short term heating rates, so allow them a lower measured heating rate
							if (actualTemperatureRise < expectedTemperatureRise * MinTemperatureRiseFactors[(unsigned int)GetFunction()])
							{
								++heaterExcursionFaultCount;
								if ((float)(heaterExcursionFaultCount * Heat::NormalHeaterPollInterval) > GetMaxHeatingFaultTime() * SecondsToMillis)
								{
									RaiseHeaterFault(HeaterFaultType::temperatureRisingTooSlowly,
														"expected %.2f" DEGREE_SYMBOL "C/sec measured %.2f" DEGREE_SYMBOL "C/sec",
															(double)expectedRate, (double)(actualTemperatureRise/actualInterval));
								}
							}
							else
							{
								lastTemperatureValue = temperature;
								lastTemperatureMillis = now;
								if (heaterExcursionFaultCount != 0)
								{
									--heaterExcursionFaultCount;
								}
							}
						}
					}
				}
#if SUPPORT_LP5817
				UpdateStatusLed();
#endif
				break;

			case HeaterMode::stable:
				// Check for maximum temperature excursion exceeded when we were at a stable temperature.
				if (   fabsf(error) > GetMaxTemperatureExcursion()
					&& (   error > 0.0											// if the temperature we are reading has dropped unexpectedly, e.g. indirect sensor can no longer see a tool
						|| temperature > MaxAmbientTemperature					// or the temperature reading is too high and greater than a reasonable ambient temperature (should we use chamber temperature instead, for a tool heater?)
					   )
				   )
				{
					++heaterExcursionFaultCount;
					if ((float)(heaterExcursionFaultCount * Heat::NormalHeaterPollInterval) > GetMaxHeatingFaultTime() * SecondsToMillis)
					{
						RaiseHeaterFault(HeaterFaultType::exceededAllowedExcursion,
											"target %.1f" DEGREE_SYMBOL "C actual %.1f" DEGREE_SYMBOL "C",
												(double)targetTemperature, (double)temperature);
					}
				}
				else if (heaterExcursionFaultCount != 0)
				{
					--heaterExcursionFaultCount;
				}
				break;

			case HeaterMode::cooling:
				if (-error <= TemperatureCloseEnough && targetTemperature > MaxAmbientTemperature)
				{
					// We have cooled to close to the target temperature, so we should now maintain that temperature
					mode = HeaterMode::stable;
					heaterExcursionFaultCount = 0;
#if CHECK_HEATER_PWM
					heaterPwmFaultCount = 0;
#endif
#if SUPPORT_LP5817
					UpdateStatusLed();
#endif
				}
				else
				{
					// We could check for temperature excessive or not falling here, but without an alarm or a power-off mechanism, there is not much we can do
					// TODO emergency stop?
				}
				break;

			default:		// this covers off, fault, suspended, and the auto tuning states
				break;
			}

			// Calculate the PWM
			if (mode <= HeaterMode::suspended)
			{
				lastPwm = 0.0;
			}
			else if (mode < HeaterMode::firstTuningMode)
			{
				// Performing normal temperature control
				if (GetModel().UsePid())
				{
					// Using PID mode. Determine the PID parameters to use.
					const bool inLoadMode = (mode == HeaterMode::stable) || (fabsf(error) < 3.0);		// use standard PID when maintaining temperature
					const PidParameters& params = GetModel().GetPidParameters(inLoadMode);

					// If the P and D terms together demand that the heater is full on or full off, disregard the I term to reduce integral windup
					const float errorMinusDterm = error - (params.tD * derivative);
					const float pPlusD = params.kP * errorMinusDterm;
					const float expectedPwm = GetModel().EstimateRequiredPwm(temperature - ambientTemperature, lastFanPwm, currentVoltage, 0.0);		//TODO pass filamentPwm
					if (pPlusD + expectedPwm > GetModel().GetMaxPwm())
					{
						lastPwm = GetModel().GetMaxPwm();
						// If we are heating up, preset the I term to the expected PWM at this temperature, ready for the switch over to PID
						if (mode == HeaterMode::heating && error > 0.0 && derivative > 0.0)
						{
							iAccumulator = expectedPwm;
						}
					}
					else if (pPlusD + expectedPwm < 0.0)
					{
						lastPwm = 0.0;
					}
					else
					{
						TaskCriticalSectionLocker lock;					// avoid a race with tasks that implement feedforward
						iAccumulator = constrain<float>
										(iAccumulator + (error * params.kP * params.recipTi * Heat::NormalHeaterPollInterval * MillisToSeconds),
											0.0, GetModel().GetMaxPwm());
						lastPwm = constrain<float>(pPlusD + iAccumulator, 0.0, GetModel().GetMaxPwm());
					}

#if CHECK_HEATER_PWM
					// The following safety check is no good for bed heaters that have a large thermal reservoir loosely coupled to the heater,
					// because the required PWM is higher than the expected value from tuning until the reservoir has heated up.
					// So we apply it to tool heaters only.
					if (mode == HeaterMode::stable && GetFunction() == HeaterFunction::tool)
					{
						const float limitedAccumulator = min<float>(iAccumulator, GetModel().GetMaxPwm());
						if (limitedAccumulator > expectedPwm * GetPwmFaultLevel())
						{
							++heaterPwmFaultCount;
							if (heaterPwmFaultCount * Heat::NormalHeaterPollInterval > GetMaxPwmFaultTime() * SecondsToMillis)
							{
								RaiseHeaterFault(HeaterFaultType::pwmTooHigh,
													"expected %.3f actual %.3f",
														(double)expectedPwm, (double)limitedAccumulator);
							}
						}
						else if (heaterPwmFaultCount != 0)
						{
							--heaterPwmFaultCount;
						}
					}
#endif
				}
				else
				{
					// Using bang-bang mode
					lastPwm = (error > 0.0) ? GetModel().GetMaxPwm() : 0.0;
				}

				// Check if the generated PWM signal needs to be inverted for inverse temperature control
				if (GetModel().IsInverted())
				{
					lastPwm = GetModel().GetMaxPwm() - lastPwm;
				}

				// Verify that everything is operating in the required temperature range
				for (size_t i = 0; i < ARRAY_SIZE(monitors); ++i)
				{
					HeaterMonitor& prot = monitors[i];
					if (!prot.Check(GetMaxBadTemperatureCount()))
					{
						lastPwm = 0.0;
						switch (prot.GetAction())
						{
						case HeaterMonitorAction::GenerateFault:
							RaiseHeaterFault(HeaterFaultType::monitorTriggered, "monitor %u was triggered", i);
							break;

						case HeaterMonitorAction::TemporarySwitchOff:
							// Do nothing, the PWM value has already been set above
							break;

						case HeaterMonitorAction::PermanentSwitchOff:
							SwitchOff();
							break;
						}
					}
				}
			}
			else
			{
				DoTuningStep();
			}
		}
		else
		{
			lastPwm = 0.0;
		}

		// Set the heater power and update the average PWM
		SetHeater(lastPwm);
		constexpr float avgFactor = Heat::NormalHeaterPollInterval/(HeatPwmAverageTime * SecondsToMillis);
		averagePWM = (averagePWM * (1.0 - avgFactor)) + (lastPwm * avgFactor);

		// For temperature sensors which do not require frequent sampling and averaging,
		// their temperature is read here and error/safety handling performed.  However,
		// unlike the Tick ISR, this code is not executed at interrupt level and consequently
		// runs the risk of having undesirable delays between calls.  To guard against this,
		// we record for each PID object when it was last sampled and have the Tick ISR
		// take action if there is a significant delay since the time of last sampling.
		lastSampleTime = millis();

//  	debugPrintf("Heater %d: e=%f, P=%f, I=%f, d=%f, r=%f\n", heater, error, pp.kP*error, temp_iState, temp_dState, result);
	}
}

void LocalHeater::ResetFault() noexcept
{
	badTemperatureCount = 0;
	if (mode == HeaterMode::fault)
	{
#if SUPPORT_LP5817
		Platform::GetStatusLedControl()->ClearError(LedStatusCode::heatingFault);
#endif
		mode = HeaterMode::off;
		SwitchOff();
	}
}

float LocalHeater::GetAveragePWM() const noexcept
{
	return averagePWM;
}

// Get a conservative estimate of the expected heating rate at the current temperature and specified PWM when trhere is no filament cooling. The result may be negative.
float LocalHeater::GetExpectedHeatingRate(float voltage) const noexcept
{
	const float temperatureRise = max<float>(temperature - LowAmbientTemperature, 0.0);
	const float pwm = min<float>(GetAveragePWM(), lastPwm);
	return GetModel().GetExpectedHeatingRate(temperatureRise, 1.0, pwm, voltage, 0.0);
}

// Start or stop running heater tuning cycles
GCodeResult LocalHeater::TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply) noexcept
{
	if (msg.on)
	{
		if (lastPwm > 0.0 || GetAveragePWM() > 0.02)
		{
			reply.printf("heater %u must be off and cold before auto tuning it", GetHeaterNumber());
			return GCodeResult::error;
		}

		// We could do some more checks here but the main board should have done all the checks needed already
		tuningHighTemp = msg.highTemp;
		tuningLowTemp = msg.lowTemp;
		tuningPwm = msg.pwm;
		tuningPeakTempDrop = msg.peakTempDrop;
		timeSetHeating = millis();
		tuningCycleComplete = false;
		cyclesDone = 0;
		mode = HeaterMode::tuning1;
	}
	else
	{
		SwitchOff();
	}
	return GCodeResult::ok;
}

// Adjust heater power for fan PWM or extrusion change
GCodeResult LocalHeater::ApplyFeedForward(const CanMessageHeaterFeedForwardV1& msg, const StringRef& reply) noexcept
{
	if (mode == HeaterMode::stable)
	{
		float pwmBoost = msg.extrusionPwmBoost - lastExtrusionPwmBoost;
		lastExtrusionPwmBoost = msg.extrusionPwmBoost;
		if (msg.fanPwmFraction != lastFanPwm)
		{
			const float oldFanPwm = lastFanPwm;
			lastFanPwm = msg.fanPwmFraction;
			pwmBoost += GetModel().GetPwmCorrectionForFan(GetTargetTemperature() - ambientTemperature, oldFanPwm, msg.fanPwmFraction) * FanFeedForwardMultiplier;
		}
		TaskCriticalSectionLocker lock;
		iAccumulator += pwmBoost;
	}
	extrusionTemperatureBoost = msg.extrusionTemperatureBoost;
	return GCodeResult::ok;
}

// This is called on each temperature sample when auto tuning
// It must set lastPWM to the required PWM, unless it is the same as last time.
void LocalHeater::DoTuningStep() noexcept
{
	const uint32_t now = millis();
	switch (mode)
	{
	case HeaterMode::tuning1:		// Heating up
		if (temperature >= tuningHighTemp)							// if reached target
		{
			// Move on to next phase
			lastPwm = 0.0;
			SetHeater(0.0);
			peakTemp = afterPeakTemp = temperature;
			lastOffTime = peakTime = afterPeakTime = now;
			mode = HeaterMode::tuning2;
		}
		else
		{
			lastPwm = tuningPwm;
		}
		return;

	case HeaterMode::tuning2:		// Heater is off, record the peak temperature and time
		if (temperature >= peakTemp)
		{
			peakTemp = afterPeakTemp = temperature;
			peakTime = afterPeakTime = now;
		}
		else if (temperature < tuningLowTemp)
		{
			// Temperature has dropped below the low limit.
			// If we have been doing idle cycles, see whether we can switch to collecting data, and turn the heater on.
			// If we have been collecting data, see if we have enough, and either turn the heater on to start another cycle or finish tuning.

			// Save the data (don't know whether we need it yet)
			dHigh = peakTime - lastOffTime;
			tOff = now - lastOffTime;
			coolingRate = (afterPeakTemp - temperature) * SecondsToMillis/(now - afterPeakTime);
			lastOnTime = peakTime = afterPeakTime = now;
			peakTemp = afterPeakTemp = temperature;
			lastPwm = tuningPwm;						// turn on heater at specified power
			mode = HeaterMode::tuning3;
		}
		else if (afterPeakTime == peakTime && tuningHighTemp - temperature >= tuningPeakTempDrop)
		{
			afterPeakTime = now;
			afterPeakTemp = temperature;
		}
		return;

	case HeaterMode::tuning3:	// Heater is turned on, record the lowest temperature and time
		if (temperature <= peakTemp)
		{
			peakTemp = afterPeakTemp = temperature;
			peakTime = afterPeakTime = now;
		}
		else if (temperature >= tuningHighTemp)
		{
			// We have reached the target temperature, so record a data point and turn the heater off
#if HAS_VOLTAGE_MONITOR
			tuningVoltage = Platform::GetCurrentVinVoltage();	// save this while the heater is on
#else
			tuningVoltage = 0.0;
#endif
			dLow = peakTime - lastOnTime;
			tOn = now - lastOnTime;
			heatingRate = (temperature - afterPeakTemp) * SecondsToMillis/(now - afterPeakTime);
			lastOffTime = peakTime = afterPeakTime = now;
			peakTemp = afterPeakTemp = temperature;
			lastPwm = 0.0;										// turn heater off
			mode = HeaterMode::tuning2;
			++cyclesDone;
			tuningCycleComplete = true;
		}
		else if (afterPeakTime == peakTime && temperature - tuningLowTemp >= tuningPeakTempDrop)
		{
			afterPeakTime = now;
			afterPeakTemp = temperature;
		}
		return;

	default:
		// Should not happen, but if it does then quit
		break;
	}

	// If we get here, we have finished
	SwitchOff();								// sets mode and lastPWM, also deletes tuningTempReadings
}

// Suspend the heater, or resume it
void LocalHeater::Suspend(bool sus) noexcept
{
	if (sus)
	{
		if (mode == HeaterMode::stable || mode == HeaterMode::heating || mode == HeaterMode::cooling)
		{
			mode = HeaterMode::suspended;
			SetHeater(0.0);
			lastPwm = 0.0;
		}
	}
	else if (mode == HeaterMode::suspended)
	{
		String<1> dummy;
		SwitchOn(dummy.GetRef());
	}
}

// Get a heater tuning cycle report, if we have one. Caller must fill in the heater number.
/*static*/ bool LocalHeater::GetTuningCycleData(CanMessageHeaterTuningReport& msg) noexcept
{
	if (tuningCycleComplete)
	{
		msg.cyclesDone = cyclesDone;
		msg.dhigh = dHigh;
		msg.dlow = dLow;
		msg.ton = tOn;
		msg.toff = tOff;
		msg.heatingRate = heatingRate;
		msg.coolingRate = coolingRate;
		msg.voltage = tuningVoltage;
		tuningCycleComplete = false;
		return true;
	}

	return false;
}

// Raise a heater fault. This turns off the heater, sets its state to 'fault', and sends an event to the main board.
// The length of text to be included must not exceed 55 characters + terminator, else it will be truncated.
void LocalHeater::RaiseHeaterFault(HeaterFaultType type, const char *_ecv_array format, ...) noexcept
{
	lastPwm = 0.0;
	SetHeater(0.0);
	if (mode != HeaterMode::fault)
	{
		mode = HeaterMode::fault;
		va_list vargs;
		va_start(vargs, format);
		CanInterface::RaiseEvent(EventType::heater_fault, (uint16_t)type, GetHeaterNumber(), format, vargs);
		va_end(vargs);
#if SUPPORT_LP5817
		Platform::GetStatusLedControl()->SetErrorOrTransient(LedStatusCode::heatingFault);
#endif
	}
}

#if SUPPORT_LP5817

void LocalHeater::UpdateStatusLed() noexcept
{
	switch (mode)
	{
	case HeaterMode::heating:
		Platform::GetStatusLedControl()->SetStatus(LedStatusCode::heating, constrain<float>((temperature - 25.0)/(GetTargetTemperature() - 25.0), 0.0, 1.0));
		break;

	case HeaterMode::cooling:
		Platform::GetStatusLedControl()->SetStatus(LedStatusCode::cooling);
		break;

	case HeaterMode::stable:
		Platform::GetStatusLedControl()->SetStatus(LedStatusCode::atTarget);
		break;

	case HeaterMode::off:
		Platform::GetStatusLedControl()->SetStatus(LedStatusCode::cold);
		break;

	//TODO what about the tuning states?
	default:
		break;
	}
}

#endif

// End
