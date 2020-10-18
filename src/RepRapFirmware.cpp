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

extern const char VersionText[] = "Duet " BOARD_TYPE_NAME " firmware version " VERSION;

Move *moveInstance;

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
const PinDescriptionBase *AppGetPinDescription(Pin p) noexcept
{
	return (p < ARRAY_SIZE(PinTable)) ? &PinTable[p] : nullptr;
}

// Define replacement standard library functions
#include <syscalls.h>

// End
