/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAME70

#include <Version.h>

#if defined(MB6HC)
extern const char VersionText[] = "Duet 3 MB6HC CAN IAP version " VERSION_TEXT;
#elif defined(MB6XD)
extern const char VersionText[] = "Duet 3 MB6XD CAN IAP version " VERSION_TEXT;
#endif

void DeviceInit() noexcept
{
}

#endif

// End
