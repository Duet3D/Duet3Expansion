/*
 * RepRapFirmware.cpp
 *
 *  Created on: 10 Sep 2018
 *      Author: David
 */

#include "RepRapFirmware.h"
#include "Movement/Move.h"
#include "GCodes/GCodes.h"
#include "Heating/Heat.h"
#include "InputMonitors/InputMonitor.h"
#include "Platform.h"
#include "CommandProcessing/CommandProcessor.h"
#include "RTOSIface/RTOSIface.h"

Move *moveInstance;

namespace RepRap
{
	static unsigned int heatTaskIdleTicks = 0;
	static bool resetting = false;

	void Init()
	{
		Platform::Init();
		GCodes::Init();
		Heat::Init();
		InputMonitor::Init();
		moveInstance = new Move();
		moveInstance->Init();
	}

	void Spin()
	{
		Platform::Spin();
		GCodes::Spin();
		CommandProcessor::Spin();
		moveInstance->Spin();
//		//RTOSIface::Yield();
//		delay(1);
	}

	void Tick()
	{
		if (!resetting)
		{
			Platform::Tick();
			++heatTaskIdleTicks;
#if 0
			const bool heatTaskStuck = (heatTaskIdleTicks >= MaxTicksInSpinState);
			if (heatTaskStuck || ticksInSpinState >= MaxTicksInSpinState)		// if we stall for 20 seconds, save diagnostic data and reset
			{
				resetting = true;
				for (size_t i = 0; i < MaxHeaters; i++)
				{
					Platform::SetHeater(i, 0.0);
				}
				Platform::DisableAllDrives();

				// We now save the stack when we get stuck in a spin loop
				__asm volatile("mrs r2, psp");
				register const uint32_t * stackPtr asm ("r2");					// we want the PSP not the MSP
				Platform::SoftwareReset(
					(heatTaskStuck) ? (uint16_t)SoftwareResetReason::heaterWatchdog : (uint16_t)SoftwareResetReason::stuckInSpin,
					stackPtr + 5);												// discard uninteresting registers, keep LR PC PSR
			}
#endif
		}
	}
}

void debugPrintf(const char* fmt, ...)
{
	va_list vargs;
	va_start(vargs, fmt);
	Platform::MessageF(DebugMessage, fmt, vargs);
	va_end(vargs);
}

// class MillisTimer members

// Start or restart the timer
void MillisTimer::Start()
{
	whenStarted = millis();
	running = true;
}

// Check whether the timer is running and a timeout has expired, but don't stop it
bool MillisTimer::Check(uint32_t timeoutMillis) const
{
	return running && millis() - whenStarted >= timeoutMillis;
}

// Check whether a timeout has expired and stop the timer if it has, else leave it running if it was running
bool MillisTimer::CheckAndStop(uint32_t timeoutMillis)
{
	const bool ret = Check(timeoutMillis);
	if (ret)
	{
		running = false;
	}
	return ret;
}

// Return a pointer to the pin description entry. Declared in and called from CoreN2G.
const PinDescriptionBase *GetPinDescription(Pin p) noexcept
{
	return (p < ARRAY_SIZE(PinTable)) ? &PinTable[p] : nullptr;
}

// Define replacement standard library functions
#include <syscalls.h>

// End
