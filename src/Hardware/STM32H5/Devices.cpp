/*
 * Devices.cpp
 *
 *  Created on: 6 Oct 2026
 *      Author: David
 */

#include <Hardware/Devices.h>

#if STM32H5

#include <Version.h>

extern const char VersionText[] = "STM32H5 bootloader version " VERSION_TEXT;

void DeviceInit() noexcept
{
	// When we have multiple boards using the STM32H5 we will need to initialise the ADC, but for now we haver only one board type
}

#endif

// End
