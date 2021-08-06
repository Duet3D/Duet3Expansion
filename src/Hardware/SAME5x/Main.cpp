#include <CoreIO.h>

#if SAME5x

void AppInit() noexcept
{
	// We use the standard clock configuration, so nothing needed here
}

// Return the XOSC frequency in MHz
unsigned int AppGetXoscFrequency() noexcept
{
#ifdef EXP3HC
	return 0;		// auto detect 12 or 25MHz
#else
	return 25;		// other boards (only EXP1HCL at present) always use 25MHz
#endif
}

// Return the XOSC number
unsigned int AppGetXoscNumber() noexcept
{
	return 0;
}

#endif

// End
