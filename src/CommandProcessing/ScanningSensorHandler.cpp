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

static LDC1612 *sensor = nullptr;

void ScanningSensorHandler::Init() noexcept
{
	// Set up the external clock to the LDC1612.
	// The higher the better, but the maximum is 40MHz. We use the 96MHz DPLL output divided by 3 to get 32MHz.
	ConfigureGclk(GclkNumPA23, GclkSource::dpll, 3, true);
	SetPinFunction(LDC1612ClockGenPin, GpioPinFunction::H);

	sensor = new LDC1612(Platform::GetSharedI2C());

	if (sensor->CheckPresent())
	{
		sensor->SetDefaultConfiguration(0);
	}
	else
	{
		DeleteObject(sensor);
		ClearPinFunction(LDC1612ClockGenPin);
	}
}

bool ScanningSensorHandler::IsPresent() noexcept
{
	return sensor != nullptr;
}

uint32_t ScanningSensorHandler::GetReading() noexcept
{
	if (sensor != nullptr)
	{
		uint32_t val;
		if (sensor->GetChannelResult(0, val))
		{
			if ((val >> 28) == 0)				// if no error
			{
				return val;						// return all 28 bits of result
			}
		}
	}
	return 0;
}

void ScanningSensorHandler::AppendDiagnostics(const StringRef& reply) noexcept
{
	reply.lcat("Inductive sensor: ");
	if (IsPresent())
	{
		sensor->AppendDiagnostics(reply);
	}
	else
	{
		reply.cat("not found");
	}
}

#endif

// End
