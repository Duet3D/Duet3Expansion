/*
 * Main.cpp
 *
 *  Created on: 27 Jul 2020
 *      Author: David
 */

#include <CoreIO.h>

#if SAMC21

void AppInit() noexcept
{
	// We use the standard clock configuration, so nothing needed here
}

// Return the XOSC frequency in MHz
unsigned int AppGetXoscFrequency() noexcept
{
#ifdef SAMMYC21
	return 16;
#else
	return 0;			// auto detect 12 or 25MHz
#endif
}

#endif

// End
