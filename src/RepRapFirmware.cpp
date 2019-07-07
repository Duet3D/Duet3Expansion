/*
 * RepRapFirmware.cpp
 *
 *  Created on: 10 Sep 2018
 *      Author: David
 */

#include "RepRapFirmware.h"
#include "Movement/Move.h"
#include "GCodes/GCodes.h"
#include "Platform.h"
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
		moveInstance = new Move();
		moveInstance->Init();
	}

	void Spin()
	{
		Platform::Spin();
		GCodes::Spin();
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

// End
