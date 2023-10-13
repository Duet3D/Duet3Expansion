/*
 * ScanningSensorHandler.cpp
 *
 *  Created on: 16 Jun 2023
 *      Author: David
 */

#include "ScanningSensorHandler.h"

#if SUPPORT_LDC1612

#include <Hardware/LDC1612.h>
#include <Platform/Platform.h>
#include <CanMessageFormats.h>
#include <AnalogIn.h>

constexpr unsigned int ResultBitsDropped = 8;		// we drop this number of least significant bits in the result
constexpr uint32_t BadReadingVal = 999999;			// close to 2 ^ (28 - resultBitsDropped)

static LDC1612 *sensor = nullptr;
static AnalogIn::AdcTaskHookFunction *oldHookFunction = nullptr;
static volatile uint32_t lastReading = 0;
static volatile bool isCalibrating = false;

static void LDC1612TaskHook() noexcept
{
	// The LDC1612 generates lots of bus errors if we try to read the data when no new data is available
	if (!isCalibrating && sensor->IsChannelReady(0))
	{
		uint32_t val;
		if (sensor->GetChannelResult(0, val))		// if no error
		{
			lastReading = val;						// save all 28 bits of data + 4 error bits
		}
		else
		{
			lastReading = 0;
		}
	}

	if (oldHookFunction != nullptr)
	{
		oldHookFunction();
	}
}

void ScanningSensorHandler::Init() noexcept
{
	// Set up the external clock to the LDC1612.
	// The higher the better, but the maximum is 40MHz
#if defined(SAMMYC21) || defined(TOOL1LC)
	// Assume we are using a LDC1612 breakout board with its own crystal, so we don't need to generate a clock
#elif defined(SZP)
	// We can use the 96MHz DPLL output divided by 3 to get 32MHz but it is probably better to use 25MHz from the crystal directly for better stability.
	static_assert(LDC1612::ClockFrequency == 25.0 || LDC1612::ClockFrequency == 32.0);
	if constexpr(LDC1612::ClockFrequency == 25.0)
	{
		ConfigureGclk(GclkNumPA23, GclkSource::xosc, 1, true);
	}
	else if constexpr(LDC1612::ClockFrequency == 32.0)
	{
		ConfigureGclk(GclkNumPA23, GclkSource::dpll, 3, true);
	}
	SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::H);
#elif defined(TOOL1RR)
	// We use the 120MHz DPLL output divided by 4 to get 30MHz. It might be better to use 25MHz from the crystal directly for better stability.
	static_assert(LDC1612::ClockFrequency == 25.0 || LDC1612::ClockFrequency == 30.0);
	if constexpr(LDC1612::ClockFrequency == 25.0)
	{
		ConfigureGclk(GclkNumPB11, GclkSource::xosc0, 1, true);
	}
	else if constexpr(LDC1612::ClockFrequency == 30.0)
	{
		ConfigureGclk(GclkNumPB11, GclkSource::dpll0, 4, true);
	}
	SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::M);
#else
# error LDC support not implemented for this processor
#endif

	sensor = new LDC1612(Platform::GetSharedI2C());

	if (sensor->CheckPresent())
	{
		sensor->SetDefaultConfiguration(0);
		oldHookFunction = AnalogIn::SetTaskHook(LDC1612TaskHook);
	}
	else
	{
		DeleteObject(sensor);
#if defined(TOOL1LC) || defined(SZP)
		ClearPinFunction(LDC1612ClockGenPin);
#endif
	}
}

bool ScanningSensorHandler::IsPresent() noexcept
{
	return sensor != nullptr;
}

uint32_t ScanningSensorHandler::GetReading() noexcept
{
	return ((lastReading & 0xF0000000) == 0) ? lastReading >> ResultBitsDropped : BadReadingVal;
}

GCodeResult ScanningSensorHandler::SetOrCalibrateCurrent(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept
{
	if (sensor != nullptr)
	{
		if (param == CanMessageChangeInputMonitorNew::paramAutoCalibrateDriveLevelAndReport)
		{
			isCalibrating = true;
			const bool ok = (sensor->CalibrateDriveCurrent(0));
			isCalibrating = false;
			if (ok)
			{
				extra = sensor->GetDriveCurrent(0);
				reply.printf("Calibration successful, sensor drive current is %u", extra);
				return GCodeResult::ok;
			}
			reply.copy("failed to calibrate sensor drive current");
		}
		else if (param == CanMessageChangeInputMonitorNew::paramReportDriveLevel)
		{
			extra = sensor->GetDriveCurrent(0);
			reply.printf("Sensor drive current is %u", extra);
			return GCodeResult::ok;
		}
		else
		{
			if (param > 31)
			{
				param = 31;
			}
			isCalibrating = true;
			const bool ok = (sensor->SetDriveCurrent(0, param));
			isCalibrating = false;
			if (ok)
			{
				extra = param;
				return GCodeResult::ok;
			}
			reply.copy("failed to set sensor drive current");
		}
	}
	extra = 0xFF;
	return GCodeResult::error;
}

void ScanningSensorHandler::AppendDiagnostics(const StringRef& reply) noexcept
{
	reply.lcat("Inductive sensor: ");
	if (IsPresent())
	{
		// Append diagnostic data to string
		uint32_t val = lastReading;
		if (val != 0)
		{
			reply.catf("raw value %" PRIu32 ", frequency %.2fMHz, current setting %u",
						val & 0x0FFFFFFF, (double)ldexpf((val & 0x0FFFFFFF) * LDC1612::FRef, -28), sensor->GetDriveCurrent(0));
			if ((val >> 28) == 0)
			{
				reply.cat(", ok");
			}
			else
			{
				if ((val >> 28) & LDC1612::ERR_UR0)
				{
					reply.cat(", under-range error");
				}
				if ((val >> 28) & LDC1612::ERR_OR0)
				{
					reply.cat(", over-range error");
				}
				if ((val >> 28) & LDC1612::ERR_WD0)
				{
					reply.cat(", watchdog error");
				}
				if ((val >> 28) & LDC1612::ERR_AE0)
				{
					reply.cat(", amplitude error");
				}
			}
		}
		else
		{
			reply.cat("error retrieving data from LDC1612");
		}
	}
	else
	{
		reply.cat("not found");
	}
}

#endif

// End
