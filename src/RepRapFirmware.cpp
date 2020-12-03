/*
 * RepRapFirmware.cpp
 *
 *  Created on: 10 Sep 2018
 *      Author: David
 */

#include "RepRapFirmware.h"
#include "Platform.h"
#include "RTOSIface/RTOSIface.h"
#include "Version.h"
#include <General/SafeVsnprintf.h>

// Version text, without the build date
extern const char VersionText[] = "Duet " BOARD_TYPE_NAME " firmware version " VERSION;

Move *moveInstance;

void debugPrintf(const char* fmt, ...)
{
#ifdef DEBUG				// save on stack usage by not calling vuprintf if debugging is disabled
	va_list vargs;
	va_start(vargs, fmt);
	vuprintf(Platform::DebugPutc, fmt, vargs);
	va_end(vargs);
#endif
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
const PinDescriptionBase *AppGetPinDescription(Pin p) noexcept
{
	return (p < ARRAY_SIZE(PinTable)) ? &PinTable[p] : nullptr;
}

// Define replacement standard library functions

#if SAMC21
// Reduce the size of the system stack below the default 1024 to save memory. When we set it to 512, M122 reported just 12 words unused, so try a higher value.
# define SystemStackSize	(600)
#endif

#include <syscalls.h>

// End
