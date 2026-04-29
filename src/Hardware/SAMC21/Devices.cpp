/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAMC21

#include <RepRapFirmware.h>
#include <AnalogIn.h>
#include <Version.h>

// The interrupt vector table points to this
extern const char VersionText[] =
#ifdef SAMMYC21
	"SAMMY-C21 bootloader version " VERSION_TEXT;
#else
	"SAMC21 bootloader version " VERSION_TEXT;
#endif

void DeviceInit() noexcept
{
#ifndef SAMMYC21
	AnalogIn::Init(CommonAdcDevice);
#endif
}

#endif

// End
