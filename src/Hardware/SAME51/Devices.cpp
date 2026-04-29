/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAME5x

#include <Version.h>

#if defined(CAN_IAP)
extern const char VersionText[] = "Duet 3 Mini CAN IAP version " VERSION_TEXT;
#elif defined(COMPOSITE)
extern const char VersionText[] = "SAME5x composite bootloader version " VERSION_TEXT;
#else
extern const char VersionText[] = "SAME5x bootloader version " VERSION_TEXT;
#endif

void DeviceInit() noexcept
{
}

#endif

// End
