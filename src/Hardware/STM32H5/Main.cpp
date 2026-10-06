/*
 * Main.cpp
 *
 *  Created on: 6 Oct 2026
 *      Author: David
 */

#include <CoreIO.h>

#if STM32H5

void AppInit() noexcept
{
	// We use the standard clock configuration, so nothing needed here
}

// Return the XOSC frequency in MHz
unsigned int AppGetXoscFrequency() noexcept
{
	return 24;
}

// Return the XOSC number
unsigned int AppGetXoscNumber() noexcept
{
	return 0;
}

#endif

// End
