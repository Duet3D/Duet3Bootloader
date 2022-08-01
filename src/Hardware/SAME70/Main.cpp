#include <CoreIO.h>

#if SAME70

void AppInit() noexcept
{
	// We use the standard clock configuration, so nothing needed here
}

// Return the XOSC frequency in MHz
unsigned int AppGetXoscFrequency() noexcept
{
	return 12000;
}

#endif

// End
