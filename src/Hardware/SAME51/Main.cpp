#include <CoreIO.h>

#if SAME5x

void AppInit() noexcept
{
	// We use the standard clock configuration, so nothing needed here
}

// Return the XOSC frequency in MHz
unsigned int AppGetXoscFrequency() noexcept
{
#if defined(CAN_IAP)
	return 25;			// Duet 3 Mini always uses a 25MHz crystal
#else
	return 0;			// EXP3HC may use 16MHz or 25MHz. EXP1HCL uses 25MHz.
#endif
}

// Return the XOSC number
unsigned int AppGetXoscNumber() noexcept
{
#if defined(CAN_IAP)
	return 1;			// Duet 3 Mini uses XOSC 1
#else
	return 0;			// EXP3HC and EXP1HCL use XOSC 0
#endif
}

#endif

// End
