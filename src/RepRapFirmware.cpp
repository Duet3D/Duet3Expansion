/*
 * RepRapFirmware.cpp
 *
 *  Created on: 10 Sep 2018
 *      Author: David
 */

#include "RepRapFirmware.h"
#include <Platform/Platform.h>
#include <RTOSIface/RTOSIface.h>
#include "Version.h"
#include <General/SafeVsnprintf.h>
#include <CAN/CanInterface.h>

// Version text, without the build date. One of the unused interrupt vectors declared in CoreN2G point to this.
extern const char VersionText[] = "Duet " BOARD_TYPE_NAME " firmware version " VERSION;

#if SUPPORT_DRIVERS
Move *moveInstance;
#endif

void debugPrintf(const char* fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
#if USE_SERIAL_DEBUG
	vuprintf(Platform::DebugPutc, fmt, vargs);
#else
	vuprintf(CanInterface::DebugPutc, fmt, vargs);
#endif
	va_end(vargs);
}

void debugVprintf(const char *fmt, va_list vargs) noexcept
{
#if USE_SERIAL_DEBUG
	vuprintf(Platform::DebugPutc, fmt, vargs);
#else
	vuprintf(CanInterface::DebugPutc, fmt, vargs);
#endif
}

// class MillisTimer members

// Start or restart the timer
void MillisTimer::Start() noexcept
{
	whenStarted = millis();
	running = true;
}

// Check whether the timer is running and a timeout has expired, but don't stop it
bool MillisTimer::CheckNoStop(uint32_t timeoutMillis) const noexcept
{
	return running && millis() - whenStarted >= timeoutMillis;
}

// Check whether a timeout has expired and stop the timer if it has, else leave it running if it was running
bool MillisTimer::CheckAndStop(uint32_t timeoutMillis)
{
	const bool ret = CheckNoStop(timeoutMillis);
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

// End
