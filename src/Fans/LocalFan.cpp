/*
 * LocalFan.cpp
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#include "LocalFan.h"
#include <Hardware/IoPorts.h>
#include <Movement/StepTimer.h>
#include <Platform/Platform.h>
#include <Heating/Heat.h>
#include <Heating/Sensors/TemperatureSensor.h>

void FanInterrupt(CallbackParameter cb) noexcept
{
	static_cast<LocalFan *>(cb.vp)->Interrupt();
}

LocalFan::LocalFan(unsigned int fanNum) noexcept
	: Fan(fanNum),
	  tachoInterruptCount(0), tachoLastResetTime(0), tachoInterval(0),
	  blipping(false)
{
}

LocalFan::~LocalFan()
{
	port.WriteAnalog(0.0);
	port.Release();
	tachoPort.Release();
}

void LocalFan::ReportPortDetails(const StringRef& str) const noexcept
{
	str.printf("Fan %u", fanNumber);
	port.AppendFullDetails(str);
	if (tachoPort.IsValid())
	{
		str.cat(", tacho");
		tachoPort.AppendBasicDetails(str);
		str.catf(", %.1f pulses per revolution", (double)(StepTimer::StepClockRate * TachoMaxInterruptCount * 60.0/tachoMultiplier));
	}
	else
	{
		str.cat(", no tacho");
	}
}

// Set the hardware PWM
// If you want make sure that the PWM is definitely updated, set lastPWM negative before calling this
void LocalFan::SetHardwarePwm(float pwmVal) noexcept
{
	port.WriteAnalog(pwmVal);
}

void LocalFan::SetTachoPulsesPerRev(float ppr) noexcept
{
	tachoMultiplier = (uint32_t)(StepTimer::StepClockRate * TachoMaxInterruptCount * 60.0/ppr);
}

// Refresh the fan PWM
// If you want make sure that the PWM is definitely updated, set lastPWM negative before calling this
void LocalFan::Refresh(bool checkSensors) noexcept
{
	float reqVal;
#if 0 //TODO HAS_SMART_DRIVERS
	uint32_t driverChannelsMonitored = 0;
#endif

	if (sensorsMonitored.IsEmpty())
	{
		reqVal = (val <= 0.0) ? 0.0 : max<float>(val * maxVal, minVal);		// scale the requested PWM by the maximum, enforce the minimum
	}
	else if (!checkSensors)
	{
		reqVal = lastVal;
	}
	else
	{
		reqVal = 0.0;
		const bool bangBangMode = (triggerTemperatures[1] <= triggerTemperatures[0]);
		sensorsMonitored.Iterate
		([&reqVal, bangBangMode, this
#if 0 // HAS_SMART_DRIVERS
		  , &driverChannelsMonitored
#endif
		 ](unsigned int sensorNum, unsigned int) noexcept
			{
				const auto sensor = Heat::FindSensor(sensorNum);
				if (sensor.IsNull())
				{
					reqVal = maxVal;					// sensor not found, so turn the fan fully on
				}
				else
				{
					//TODO we used to turn the fan on if the associated heater was being tuned
					float ht;
					const TemperatureError err = sensor->GetLatestTemperature(ht);
					if (err != TemperatureError::ok || ht < BadLowTemperature || ht >= triggerTemperatures[1])
					{
						reqVal = maxVal;
					}
					else if (!bangBangMode && ht >= triggerTemperatures[0])
					{
						// We already know that ht < triggerTemperatures[1], therefore unless we have NaNs it is safe to divide by (triggerTemperatures[1] - triggerTemperatures[0])
						const float newVal = minVal + (maxVal - minVal) * (ht - triggerTemperatures[0])/(triggerTemperatures[1] - triggerTemperatures[0]);
						if (newVal > reqVal)
						{
							reqVal = newVal;
						}
					}
					else if (lastVal > 0.0 && ht + ThermostatHysteresis > triggerTemperatures[0])		// if the fan is on, add a hysteresis before turning it off
					{
						const float newVal = (bangBangMode) ? maxVal : minVal;
						if (newVal > reqVal)
						{
							reqVal = newVal;
						}
					}
#if 0 //HAS_SMART_DRIVERS
					const int channel = sensor->GetSmartDriversChannel();
					if (channel >= 0)
					{
						driverChannelsMonitored.SetBit((unsigned int)channel);
					}
#endif
				}
			}
		);
	}

	if (reqVal > 0.0)
	{
		if (lastVal == 0.0)
		{
			// We are turning this fan on
#if 0 //TODO HAS_SMART_DRIVERS
			if (driverChannelsMonitored != 0)
			{
				reprap.GetPlatform().DriverCoolingFansOnOff(driverChannelsMonitored, true);		// tell Platform that we have started a fan that cools drivers
			}
#endif
			if (reqVal < 1.0 && blipTime != 0)
			{
				// Starting the fan from standstill, so blip the fan
				blipping = true;
				blipStartTime = millis();
			}
		}
		else if (blipping && millis() - blipStartTime >= blipTime)
		{
			blipping = false;
		}
	}
	else
	{
		blipping = false;
#if 0 //TODO HAS_SMART_DRIVERS
		if (driverChannelsMonitored != 0 && lastVal != 0.0)
		{
			reprap.GetPlatform().DriverCoolingFansOnOff(driverChannelsMonitored, false);	// tell Platform that we have stopped a fan that cools drivers
		}
#endif
	}

	lastVal = reqVal;
	SetHardwarePwm((blipping) ? 1.0 : reqVal);
}

bool LocalFan::UpdateFanConfiguration(const StringRef& reply) noexcept
{
	Refresh(true);
	return true;
}

// Update the fan if necessary. Return true if it is a thermostatic fan and is running.
bool LocalFan::Check(bool checkSensors) noexcept
{
	Refresh(checkSensors);
	return !sensorsMonitored.IsEmpty() && lastVal != 0.0;
}

bool LocalFan::AssignPorts(const char *pinNames, const StringRef& reply) noexcept
{
	IoPort* const ports[] = { &port, &tachoPort };
	const PinAccess access1[] = { PinAccess::pwm, PinAccess::read};
	if (IoPort::AssignPorts(pinNames, reply, PinUsedBy::fan, 2, ports, access1) == 0)
	{
		const PinAccess access2[] = { PinAccess::write0, PinAccess::read};
		if (IoPort::AssignPorts(pinNames, reply, PinUsedBy::fan, 2, ports, access2) == 0)
		{
			return false;
		}
	}

	// Tacho initialisation
	if (tachoPort.IsValid())
	{
		tachoPort.AttachInterrupt(FanInterrupt, InterruptMode::falling, CallbackParameter(this));
	}

	Refresh(true);
	return true;
}

// Tacho support
int32_t LocalFan::GetRPM() noexcept
{
	// The ISR sets fanInterval to the number of step interrupt clocks it took to get fanMaxInterruptCount interrupts.
	// We get 2 tacho pulses per revolution, hence 2 interrupts per revolution.
	// When the fan stops, we get no interrupts and fanInterval stops getting updated. We must recognise this and return zero.
	return (!tachoPort.IsValid())
			? -1																			// we return -1 if there is no tacho configured
			: (tachoInterval != 0 && StepTimer::GetTimerTicks() - tachoLastResetTime < 3 * StepTimer::StepClockRate)	// if we have a reading and it is less than 3 seconds old
			  ? tachoMultiplier/tachoInterval												// then calculate RPM
			  : 0;																			// else assume fan is off or tacho not connected
}

void LocalFan::Interrupt() noexcept
{
	++tachoInterruptCount;
	if (tachoInterruptCount == TachoMaxInterruptCount)
	{
		const uint32_t now = StepTimer::GetTimerTicks();
		tachoInterval = now - tachoLastResetTime;
		tachoLastResetTime = now;
		tachoInterruptCount = 0;
	}
}

// End
