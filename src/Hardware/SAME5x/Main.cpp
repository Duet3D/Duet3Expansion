#include <CoreIO.h>

#if SAME5x

void AppInit() noexcept
{
	// We use the standard clock configuration, so nothing needed here
}

// Return the XOSC frequency in MHz
unsigned int AppGetXoscFrequency() noexcept
{
	return 0;		// auto detect 12 or 25MHz
}

// Return the XOSC number
unsigned int AppGetXoscNumber() noexcept
{
	return 0;
}

#endif

// End
